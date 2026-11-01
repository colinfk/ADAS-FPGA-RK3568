#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kref.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/uaccess.h>

#define FRAME_BYTES (1920U * 1080U * 2U)
/* size_t so the dma_alloc_coherent/%zu uses agree on 32- and 64-bit builds. */
#define FRAME_ALLOC_BYTES ((size_t)PAGE_ALIGN(FRAME_BYTES))
#define CONTROL 0x1c0
#define COMPLETION_LO 0x1d0
#define COMPLETION_HI 0x1e0
#define COMPLETION_VALID BIT(31)
#define COMPLETION_BUFFER BIT(30)
#define SEQUENCE_MASK 0x3fffffffU
#define DMA_STATUS 0xff70
#define DMA_STATUS_MAGIC 0x444d4135U
#define DMA_STATUS_ACTIVE BIT(0)
#define DMA_STATUS_STOPPING BIT(1)
#define DMA_STATUS_SOURCE_IDLE BIT(2)
struct adas_control_status { u32 magic, frame_seq, command_seq, flags; };

/* Coherent completion markers stay uncached. Image pages use explicit DMA
 * ownership so CPU reads can be cached on non-coherent RK3568 PCIe hosts.
 * cached_frames=0 retains the original coherent image-buffer path. */
static unsigned int start_marker_fallbacks;
module_param(start_marker_fallbacks, uint, 0444);
MODULE_PARM_DESC(start_marker_fallbacks, "Valid ordered completions accepted despite a skipped START command acknowledgement");
static bool cached_frames = true;
module_param(cached_frames, bool, 0444);
MODULE_PARM_DESC(cached_frames, "Use cached image pages with explicit DMA synchronization");

struct adas_video_info {
	__u32 width, height, pitch, buffer_count;
	__u64 frame_bytes;
};
struct adas_video_frame {
	__u32 last_seq, seq, buffer, reserved;
};
#define IOC_START _IO('A', 1)
#define IOC_WAIT _IOWR('A', 2, struct adas_video_frame)
#define IOC_INFO _IOR('A', 3, struct adas_video_info)
#define IOC_STOP _IO('A', 4)

struct adas_video_dev {
	struct pci_dev *pdev;
	void __iomem *bar0, *bar1;
	void *cpu_addr[2];
	struct page *frame_pages[2];
	bool cpu_owned[2];
	u32 pending_buffer;
	dma_addr_t dma_addr[2];
	u32 *completion;
	dma_addr_t completion_dma;
	struct miscdevice misc;
	struct mutex lock;
	struct kref refs;
	bool opened, removed, faulted, inflight;
	u32 last_seq;
	u32 command_seq;
	bool have_seq;
};

static int adas_wait_completion(struct adas_video_dev *d, u32 *marker)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(2000);
	do {
		*marker = READ_ONCE(*d->completion);
		if (*marker & COMPLETION_VALID) {
			dma_rmb();
			return 0;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	return -ETIMEDOUT;
}

/* Read a coherent control page despite four separate PCIe MMIO reads. */
static int adas_read_control(struct adas_video_dev *d, struct adas_control_status *s)
{
    struct adas_control_status again;
    int attempt;
    for (attempt = 0; attempt < 8; attempt++) {
        memcpy_fromio(s, d->bar0 + DMA_STATUS, sizeof(*s));
        memcpy_fromio(&again, d->bar0 + DMA_STATUS, sizeof(again));
        if (!memcmp(s, &again, sizeof(*s)) && s->magic == DMA_STATUS_MAGIC &&
            (s->flags >> 24) == 5)
            return 0;
    }
    return -EAGAIN;
}

static int adas_check_completion(struct adas_video_dev *d, u32 marker)
{
    u32 seq = marker & SEQUENCE_MASK;
    if (!(marker & COMPLETION_VALID) ||
        (!!(marker & COMPLETION_BUFFER) != d->pending_buffer) ||
        (d->have_seq && seq != ((d->last_seq + 1U) & SEQUENCE_MASK)))
        return -EIO;
    return 0;
}

static int adas_consume_completion(struct adas_video_dev *d, u32 marker)
{
    int ret = adas_check_completion(d, marker);
    if (ret)
        return ret;
    if (d->frame_pages[d->pending_buffer] && !d->cpu_owned[d->pending_buffer]) {
        dma_sync_single_for_cpu(&d->pdev->dev, d->dma_addr[d->pending_buffer],
                                FRAME_ALLOC_BYTES, DMA_FROM_DEVICE);
        d->cpu_owned[d->pending_buffer] = true;
    }
    d->last_seq = marker & SEQUENCE_MASK;
    d->have_seq = true;
    d->inflight = false;
    return 0;
}

static int adas_stop(struct adas_video_dev *d)
{
    struct adas_control_status status;
    unsigned long deadline = jiffies + msecs_to_jiffies(2000);
    u32 marker, expected;
    int ret;
    ret = adas_read_control(d, &status);
    if (ret)
        goto failed;
    expected = status.command_seq + 1U;
    iowrite32(0, d->bar1 + CONTROL);
    /* Both successful and cancelled grants send an ordered MWr marker.
     * Consuming it also advances the sequence across process restarts. */
    if (d->inflight) {
        ret = adas_wait_completion(d, &marker);
        if (!ret)
            ret = adas_consume_completion(d, marker);
        if (ret)
            goto failed;
    }
    deadline = jiffies + msecs_to_jiffies(2000);
    do {
        ret = adas_read_control(d, &status);
        if (!ret && status.command_seq == expected &&
            !(status.flags & (DMA_STATUS_ACTIVE | DMA_STATUS_STOPPING)) &&
            (status.flags & DMA_STATUS_SOURCE_IDLE)) {
            d->command_seq = status.command_seq;
            d->faulted = false;
            return 0;
        }
        usleep_range(1000, 2000);
    } while (time_before(jiffies, deadline));
    ret = -ETIMEDOUT;
failed:
    d->faulted = true;
    dev_err(&d->pdev->dev, "DMA stop/quiesce failed (%d); buffers remain unavailable until recovery\n", ret);
    return ret;
}

static int adas_start(struct adas_video_dev *d)
{
	struct adas_control_status status;
	unsigned long deadline;
	u32 expected;
	int i, ret;
	if (d->faulted)
		return -EIO;
	if (d->inflight)
		return -EBUSY;
	ret = adas_read_control(d, &status);
	if (ret)
		return ret;
	if ((status.flags & (DMA_STATUS_ACTIVE | DMA_STATUS_STOPPING)) ||
	    !(status.flags & DMA_STATUS_SOURCE_IDLE))
		return -EBUSY;
	d->command_seq = status.command_seq;
	expected = status.command_seq + 1U;
	/* DMA5 starts at buffer 0 and toggles after every ordered completion,
	 * including cancelled grants. Its published sequence determines the next
	 * target even when a new process or driver attaches mid-stream. Only that
	 * buffer changes owner; the other one may still be displayed by the CPU. */
	d->pending_buffer = status.frame_seq & 1U;
	if (d->frame_pages[d->pending_buffer] && d->cpu_owned[d->pending_buffer]) {
		dma_sync_single_for_device(&d->pdev->dev, d->dma_addr[d->pending_buffer],
					   FRAME_ALLOC_BYTES, DMA_FROM_DEVICE);
		d->cpu_owned[d->pending_buffer] = false;
	}
	WRITE_ONCE(*d->completion, 0);
	dma_wmb();
	for (i = 0; i < 2; i++) {
		iowrite32(lower_32_bits(d->dma_addr[i]), d->bar1 + 0x180 + i * 0x20);
		iowrite32(upper_32_bits(d->dma_addr[i]), d->bar1 + 0x190 + i * 0x20);
	}
	iowrite32(lower_32_bits(d->completion_dma), d->bar1 + COMPLETION_LO);
	iowrite32(upper_32_bits(d->completion_dma), d->bar1 + COMPLETION_HI);
	wmb();
	iowrite32(1, d->bar1 + CONTROL);
	d->inflight = true;
	deadline = jiffies + msecs_to_jiffies(2000);
	do {
        u32 marker;
        ret = adas_read_control(d, &status);
        marker = READ_ONCE(*d->completion);
        /* The ordered completion is the DMA fence. A repeated control write
         * can advance command_seq twice while producing only one frame grant.
         * Validate its buffer/sequence before accepting this stronger proof;
         * leave inflight set so WAIT still synchronizes and consumes it. */
        if (marker & COMPLETION_VALID) {
            int checked;
            dma_rmb();
            checked = adas_check_completion(d, marker);
            if (checked) {
                d->faulted = true;
                return checked;
            }
            if (!ret) {
                d->command_seq = status.command_seq;
                if (status.command_seq != expected)
                    start_marker_fallbacks++;
            }
            return 0;
        }
		if (!ret && status.command_seq == expected) {
			d->command_seq = expected;
			if ((status.flags & DMA_STATUS_ACTIVE) ||
			    (READ_ONCE(*d->completion) & COMPLETION_VALID))
				return 0;
			d->inflight = false;
			return -EBUSY;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, deadline));
	d->faulted = true;
	return -ETIMEDOUT;
}

static long adas_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct adas_video_dev *d = file->private_data;
	struct adas_video_info info = {1920, 1080, 3840, 2, FRAME_BYTES};
	struct adas_video_frame frame = {};
	u32 marker;
	long ret = 0;
	if (mutex_lock_interruptible(&d->lock))
		return -ERESTARTSYS;
	if (d->removed) {
		ret = -ENODEV;
		goto out;
	}
	switch (cmd) {
	case IOC_INFO:
		ret = copy_to_user((void __user *)arg, &info, sizeof(info)) ? -EFAULT : 0;
		break;
	case IOC_START:
		ret = adas_start(d);
		break;
	case IOC_WAIT:
		if (!d->inflight || d->faulted) {
			ret = -EIO;
			break;
		}
		ret = adas_wait_completion(d, &marker);
		if (ret) {
			d->faulted = true;
			break;
		}
		frame.seq = marker & 0x3fffffffU;
		frame.last_seq = d->have_seq ? d->last_seq :
				 ((frame.seq - 1U) & 0x3fffffffU);
		if (adas_check_completion(d, marker)) {
			d->faulted = true;
			ret = -EIO;
			break;
		}
		frame.buffer = !!(marker & COMPLETION_BUFFER);
		/* Publish CPU ownership before returning WAIT. EFAULT still leaves the
		 * grant pending for STOP to consume, but must not skip synchronization. */
		if (d->frame_pages[frame.buffer] && !d->cpu_owned[frame.buffer]) {
			dma_sync_single_for_cpu(&d->pdev->dev, d->dma_addr[frame.buffer],
						FRAME_ALLOC_BYTES, DMA_FROM_DEVICE);
			d->cpu_owned[frame.buffer] = true;
		}
		ret = copy_to_user((void __user *)arg, &frame, sizeof(frame)) ? -EFAULT : 0;
		if (!ret) {
			ret = adas_consume_completion(d, marker);
		}
		break;
	case IOC_STOP:
		ret = adas_stop(d);
		break;
	default:
		ret = -ENOTTY;
	}
out:
	mutex_unlock(&d->lock);
	return ret;
}

static int adas_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct adas_video_dev *d = file->private_data;
	unsigned int index;
	int ret;
	/* Check page offsets before shifting; reject wraparound selectors. */
	if (vma->vm_end - vma->vm_start != FRAME_ALLOC_BYTES ||
	    (vma->vm_pgoff != 0 && vma->vm_pgoff !=
	     (FRAME_ALLOC_BYTES >> PAGE_SHIFT)))
		return -EINVAL;
	if (!(vma->vm_flags & VM_SHARED) || (vma->vm_flags & VM_WRITE))
		return -EACCES;
	index = vma->vm_pgoff != 0;
	mutex_lock(&d->lock);
	if (d->removed) {
		ret = -ENODEV;
	} else {
		vma->vm_pgoff = 0;
		vma->vm_flags &= ~VM_MAYWRITE; /* target kernel: 6.1 */
		if (d->frame_pages[index])
			ret = dma_mmap_pages(&d->pdev->dev, vma, FRAME_ALLOC_BYTES,
					     d->frame_pages[index]);
		else
			ret = dma_mmap_coherent(&d->pdev->dev, vma, d->cpu_addr[index],
						d->dma_addr[index], FRAME_ALLOC_BYTES);
	}
	mutex_unlock(&d->lock);
	return ret;
}

static void adas_free(struct kref *ref)
{
	struct adas_video_dev *d = container_of(ref, struct adas_video_dev, refs);
	int i;
	for (i = 0; i < 2; i++)
		if (d->frame_pages[i])
			dma_free_pages(&d->pdev->dev, FRAME_ALLOC_BYTES,
				       d->frame_pages[i], d->dma_addr[i], DMA_FROM_DEVICE);
		else if (d->cpu_addr[i])
			dma_free_coherent(&d->pdev->dev, FRAME_ALLOC_BYTES,
					  d->cpu_addr[i], d->dma_addr[i]);
	if (d->completion)
		dma_free_coherent(&d->pdev->dev, sizeof(*d->completion),
				  d->completion, d->completion_dma);
	pci_dev_put(d->pdev);
	kfree(d);
}

static int adas_open(struct inode *inode, struct file *file)
{
	struct adas_video_dev *d = container_of(file->private_data,
					      struct adas_video_dev, misc);
	int ret = 0;
	mutex_lock(&d->lock);
	if (d->removed)
		ret = -ENODEV;
	else if (d->opened)
		ret = -EBUSY;
	else if (d->faulted)
		ret = -EIO;
	else {
		kref_get(&d->refs);
		d->opened = true;
		file->private_data = d;
	}
	mutex_unlock(&d->lock);
	return ret;
}

static int adas_release(struct inode *inode, struct file *file)
{
	struct adas_video_dev *d = file->private_data;
	mutex_lock(&d->lock);
	if (!d->removed && d->inflight)
		adas_stop(d);
	d->opened = false;
	mutex_unlock(&d->lock);
	/* mmap holds the file reference, keeping buffers alive until VMA close. */
	kref_put(&d->refs, adas_free);
	return 0;
}

static const struct file_operations adas_fops = {
	.owner = THIS_MODULE, .open = adas_open, .release = adas_release,
	.unlocked_ioctl = adas_ioctl, .mmap = adas_mmap, .llseek = no_llseek,
};

/* Only restore kernel-assigned resources after claiming them. A stale 8-KiB
 * resource from another FPGA image must be re-enumerated, never enlarged here.
 * pci_bus_address() translates the CPU resource into the PCI address domain. */
static int adas_prepare_bars(struct pci_dev *pdev)
{
	u32 configured[2] = {0, 0}, expected[2];
	u16 command;
	int i, ret, attempt;
	bool repair = false;
	for (i = 0; i < 2; i++) {
		pci_bus_addr_t address = pci_bus_address(pdev, i);
		resource_size_t size = pci_resource_len(pdev, i);
		unsigned long flags = pci_resource_flags(pdev, i);
		if (!(flags & IORESOURCE_MEM) ||
		    (flags & (IORESOURCE_MEM_64 | IORESOURCE_PREFETCH | IORESOURCE_UNSET)) ||
		    size < (i ? 512 : 65536) || !address ||
		    address > U32_MAX || size - 1 > U32_MAX - address ||
		    !is_power_of_2(size) || (address & (size - 1))) {
			dev_err(&pdev->dev, "BAR%d invalid resource; load matching FPGA Flash image and re-enumerate before binding\n", i);
			return -ENODEV;
		}
		expected[i] = address;
		ret = pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0 + i * 4,
					    &configured[i]);
		if (ret || (configured[i] & ~PCI_BASE_ADDRESS_MEM_MASK)) {
			dev_err(&pdev->dev, "BAR%d inaccessible or wrong type: %#x\n", i, configured[i]);
			return -EIO;
		}
		repair |= configured[i] != expected[i];
	}
	if (!repair)
		return 0;
	ret = pci_read_config_word(pdev, PCI_COMMAND, &command);
	if (ret)
		return -EIO;
	ret = pci_write_config_word(pdev, PCI_COMMAND,
		command & ~(PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER));
	if (ret)
		return -EIO;
	{
		u16 disabled;
		ret = pci_read_config_word(pdev, PCI_COMMAND, &disabled);
		if (ret || (disabled & (PCI_COMMAND_IO | PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER)))
			return -EIO;
	}
	for (i = 0; i < 2; i++) {
		if (configured[i] == expected[i])
			continue;
		for (attempt = 0; attempt < 5; attempt++) {
			ret = pci_write_config_dword(pdev, PCI_BASE_ADDRESS_0 + i * 4,
						     expected[i]);
			usleep_range(1000, 2000);
			if (!ret)
				ret = pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0 + i * 4,
							    &configured[i]);
			if (!ret && configured[i] == expected[i])
				break;
		}
		if (attempt == 5) {
			dev_err(&pdev->dev, "BAR%d restore failed; memory decode and bus mastering left disabled\n", i);
			return -EIO;
		}
		dev_warn(&pdev->dev, "BAR%d restored to assigned PCI address %#x; check FPGA Flash/startup ordering\n",
			 i, expected[i]);
	}
	/* Recheck both BARs before enabling MMIO; do not enable DMA until probe ends. */
	for (i = 0; i < 2; i++) {
		ret = pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0 + i * 4, &configured[i]);
		if (ret || configured[i] != expected[i])
			return -EIO;
	}
	ret = pci_write_config_word(pdev, PCI_COMMAND, command & ~PCI_COMMAND_MASTER);
	if (ret)
		return -EIO;
	ret = pci_read_config_word(pdev, PCI_COMMAND, &command);
	if (ret || !(command & PCI_COMMAND_MEMORY) || (command & PCI_COMMAND_MASTER))
		return -EIO;
	return 0;
}

static int adas_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct adas_video_dev *d;
	int ret, i;
	if (PAGE_SIZE != 4096)
		return -EOPNOTSUPP;
	d = kzalloc(sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->pdev = pci_dev_get(pdev);
	kref_init(&d->refs);
	mutex_init(&d->lock);
	ret = pcim_enable_device(pdev);
	if (ret)
		goto fail;
	pci_clear_master(pdev);
	/* Reject stale small resources before enabling any video MMIO access. */
	for (i = 0; i < 2; i++) {
		if (!(pci_resource_flags(pdev, i) & IORESOURCE_MEM) ||
		    pci_resource_len(pdev, i) < (i ? 512 : 65536) ||
		    !pci_resource_start(pdev, i)) {
			dev_err(&pdev->dev, "BAR%d resource mismatch; verify FPGA Flash version and re-enumerate\n", i);
			ret = -ENODEV;
			goto fail;
		}
	}
	if (!(pci_resource_flags(pdev, 1) & IORESOURCE_MEM) ||
	    pci_resource_len(pdev, 1) < COMPLETION_HI + 4) {
		dev_err(&pdev->dev, "BAR1 must be a memory BAR of at least 0x1e4 bytes\n");
		ret = -ENODEV;
		goto fail;
	}
	ret = pcim_iomap_regions(pdev, BIT(0) | BIT(1), "adas_video");
	if (ret)
		goto fail;
	ret = adas_prepare_bars(pdev);
	if (ret)
		goto fail;
	d->bar0 = pcim_iomap_table(pdev)[0];
	d->bar1 = pcim_iomap_table(pdev)[1];
	if (!d->bar0 || !d->bar1) {
		ret = -ENOMEM;
		goto fail;
	}
	{
		struct adas_control_status status;
		unsigned long deadline = jiffies + msecs_to_jiffies(1000);
		do {
			ret = adas_read_control(d, &status);
			if (!ret)
				break;
			usleep_range(1000, 2000);
		} while (time_before(jiffies, deadline));
		if (ret) {
			dev_err(&pdev->dev, "v5 DMA5 stop protocol missing; load matching FPGA bitstream\n");
			ret = -ENODEV;
			goto fail;
		}
	}
	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
	if (ret)
		ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		goto fail;
	d->completion = dma_alloc_coherent(&pdev->dev, sizeof(*d->completion),
					   &d->completion_dma, GFP_KERNEL);
	if (!d->completion) {
		ret = -ENOMEM;
		goto fail;
	}
	*d->completion = 0;
	for (i = 0; i < 2; i++) {
		if (cached_frames) {
			d->frame_pages[i] = dma_alloc_pages(&pdev->dev, FRAME_ALLOC_BYTES,
							 &d->dma_addr[i], DMA_FROM_DEVICE, GFP_KERNEL);
			if (d->frame_pages[i]) {
				d->cpu_addr[i] = page_address(d->frame_pages[i]);
				dma_sync_single_for_cpu(&pdev->dev, d->dma_addr[i],
							FRAME_ALLOC_BYTES, DMA_FROM_DEVICE);
				d->cpu_owned[i] = true;
			}
		} else {
			d->cpu_addr[i] = dma_alloc_coherent(&pdev->dev, FRAME_ALLOC_BYTES,
							 &d->dma_addr[i], GFP_KERNEL);
		}
		if (!d->cpu_addr[i]) {
			dev_err(&pdev->dev, "Frame %d allocation failed; need two %zu-byte DMA buffers, check CMA\n",
				i, FRAME_ALLOC_BYTES);
			ret = -ENOMEM;
			goto fail;
		}
		memset(d->cpu_addr[i], 0, FRAME_ALLOC_BYTES);
	}
	d->misc.minor = MISC_DYNAMIC_MINOR;
	d->misc.name = "adas_video0";
	d->misc.fops = &adas_fops;
	d->misc.parent = &pdev->dev;
	d->misc.mode = 0600;
	pci_set_drvdata(pdev, d);
	pci_set_master(pdev);
	ret = misc_register(&d->misc);
	if (ret) {
		pci_clear_master(pdev);
		goto fail;
	}
	dev_info(&pdev->dev, "two %s 1080p RGB565 buffers with coherent DMA completion marker\n",
		 cached_frames ? "cached DMA-synchronized" : "coherent");
	return 0;
fail:
	kref_put(&d->refs, adas_free);
	return ret;
}

static void adas_remove(struct pci_dev *pdev)
{
	struct adas_video_dev *d = pci_get_drvdata(pdev);
	misc_deregister(&d->misc);
	mutex_lock(&d->lock);
	if (d->inflight)
		adas_stop(d);
	pci_clear_master(pdev);
	pci_wait_for_pending_transaction(pdev);
	d->removed = true;
	mutex_unlock(&d->lock);
	kref_put(&d->refs, adas_free);
}

static const struct pci_device_id adas_ids[] = {
	{ PCI_DEVICE(0x0755, 0x0755) }, { }
};
MODULE_DEVICE_TABLE(pci, adas_ids);
static struct pci_driver adas_driver = {
	.name = "adas_video", .id_table = adas_ids,
	.probe = adas_probe, .remove = adas_remove,
};
module_pci_driver(adas_driver);
MODULE_LICENSE("GPL");
MODULE_VERSION("5.2-marker-ack");
MODULE_DESCRIPTION("RK3568 one-frame-grant video DMA with explicit image cache ownership");
