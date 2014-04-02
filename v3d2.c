#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/dma-mapping.h>
#include <mach/vcio.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/spinlock.h>
#include <linux/jiffies.h>

#include "v3d2_internal.h"
#include "v3d2_ioctl.h"
#include "v3d.h"

static int qpu_enable(unsigned int enable);

enum JobState { idle, notEnabled, compiling, waiting, running, finished };

struct v3d2Handle {
	spinlock_t lock;
	struct MemoryReference **references;
	int maxrefs;
	struct MemoryReference *active;
	bool gotirq;
	struct resource *io;
	JobCompileRequest *activeJob;
	volatile enum JobState bs;
	volatile enum JobState rs;
	wait_queue_head_t queue;
	JobStatusPacket status;
	void *activeBinner;
	int binnerSize;
	void *activeRenderer;
	int rendererSize;
};
struct v3d2Handle *mainHandle = 0;
volatile unsigned *v3dio = 0;
const char *states[] = { "idle","not enabled","compiling","waiting","running","finished"};
struct cdev *v3d2_cdev = 0;
static dev_t characterDevice;
static struct class *v3d2_class;

// 05 23:30:41 < ssvb> basically you just use 'dma_alloc_coherent' function and it works for large memory blocks
// you need to boot with cma=64M if you want to be able to allocate large blocks
// http://stackoverflow.com/questions/14625919/allocating-a-large-dma-buffer

volatile int count = 0;


static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs) {
	int status;
	if (!v3dio) {
		printk(KERN_ERR"v3dio isnt mapped!!!\n");
		return IRQ_HANDLED;
	}
	status = v3dio[V3D_INTCTL];
	v3dio[V3D_INTCTL] = 0x7;

	if (count > 10) return IRQ_HANDLED;
	else if (count == 10) {
		printk(KERN_ERR"fail\n");
		count++;
	}
	count++;
	//printk(KERN_ERR"interupt! status %x %x %x\n",status,v3dio[V3D_CT0CS],v3dio[V3D_CT1CS]);
	if (!mainHandle) {
		printk(KERN_ERR"interupt while device not in use\n");
		return IRQ_HANDLED;
	}
	if (mainHandle->bs == running) {
		mainHandle->bs = finished;
		if (mainHandle->rs == waiting) {
			uint32_t rawrenderer = virt_to_phys(mainHandle->activeRenderer);
			mainHandle->rs = running;
			//printk(KERN_ERR"starting renderer thread %p + %d\n",rawrenderer,mainHandle->rendererSize);
			v3dio[V3D_CT1CS] = 8000;
			__asm volatile ("mcrr p15, #0, %[end], %[start], c12" : : [end] "r" (mainHandle->activeRenderer+mainHandle->rendererSize), [start] "r" (mainHandle->activeRenderer) );
			wmb();
			v3dio[V3D_CT1CA] = rawrenderer;
			wmb();
			v3dio[V3D_CT1EA] = rawrenderer + mainHandle->rendererSize;
			wmb();
		}
	}
	// FIXME, if these finish abnormally, they wont free
	if (status & 0x2) {
		mainHandle->status.binnerFinished = 1;
		kfree(mainHandle->activeBinner);
		mainHandle->activeBinner = 0;
	}
	if (status & 0x1) {
		mainHandle->status.rendererFinished = 1;
		mainHandle->rs = finished;
		kfree(mainHandle->activeRenderer);
		mainHandle->activeRenderer = 0;
		wake_up_interruptible(&mainHandle->queue);
	}
	return IRQ_HANDLED;
}
static int v3d2_open(struct inode *inode, struct file *filp) {
	int i;
	int result;
	struct v3d2Handle *handle;
	
	if (mainHandle) return -EBUSY;
	handle = kmalloc(sizeof(struct v3d2Handle),GFP_KERNEL);
	memset(handle,0,sizeof(struct v3d2Handle));
	init_waitqueue_head(&handle->queue);
	spin_lock_init(&handle->lock);
	handle->rs = idle;
	mainHandle = handle;
	printk(KERN_ERR"FD opened PID:%d\n",current->pid);
	filp->private_data = handle;
	handle->maxrefs = 128;
	handle->active = 0;
	handle->activeBinner = NULL;
	handle->io = request_mem_region(BCM2708_PERI_BASE + 0xc00000,0x1000,"v3d2");
	if (!handle->io) {
		kfree(handle);
		return -EBUSY;
	}

	handle->references = kmalloc(sizeof(struct MemoryReference)*handle->maxrefs,GFP_KERNEL);
	if (!handle->references) {
		kfree(handle);
		return -ENOMEM;
	}
	for (i=0; i<handle->maxrefs; i++) handle->references[i] = 0;

	v3dio = ioremap_nocache(BCM2708_PERI_BASE + 0xc00000,0x1000);
	if (!v3dio) {
		kfree(handle->references);
		release_mem_region(BCM2708_PERI_BASE + 0xc00000,0x1000);
		kfree(handle);
		return -EBUSY;
	}
	if (v3dio[V3D_IDENT0] == 0x02443356) {
		printk(KERN_ERR"v3d core already online\n");
	} else {
		qpu_enable(1);
		if (v3dio[V3D_IDENT0] != 0x02443356) {
			printk(KERN_ERR"cant find magic number in v3d registers\n");
		}
	}
	printk(KERN_ERR"interupt status before reset %x %x %x\n",v3dio[V3D_INTCTL],v3dio[V3D_CT0CS],v3dio[V3D_CT1CS]);
	v3dio[V3D_INTCTL] = 0x7;
	result = request_irq(INTERRUPT_3D,(irq_handler_t) irq_handler,0,"v3d2",(void*)0);
	if (result) {
		printk(KERN_ERR" error %d getting interupt\n",result);
		handle->gotirq = false;
	} else {
		handle->gotirq = true;
		v3dio[V3D_INTENA]= 0x03;
		v3dio[V3D_L2CACTL] = 0x01;
		printk(KERN_ERR"INTENA:%x\n",v3dio[V3D_INTENA]);
	}
	return 0;
}
void CheckAndFree(struct MemoryReference *ref) {
	if (ref->mmap_count) return;
	dma_free_coherent(NULL,ref->size,ref->virt,ref->physical);
	kfree(ref);
	printk(KERN_ERR"released ram\n");
}
static int v3d2_release(struct inode *inode, struct file *filp) {
	int i;
	struct v3d2Handle *handle = filp->private_data;
	v3dio[V3D_INTENA] = 0x00;
	if (handle->gotirq) free_irq(INTERRUPT_3D,(void*)0);
	qpu_enable(0);
	if (v3dio[V3D_IDENT0] == 0x02443356) {
		printk(KERN_ERR"qpu still online\n");
	} else {
		printk(KERN_ERR"qpu shut off\n");
	}
	iounmap(v3dio);
	release_mem_region(BCM2708_PERI_BASE + 0xc00000,0x1000);
	for (i=0; i<handle->maxrefs; i++) {
		if (handle->references[i]) {
			handle->references[i]->mmap_count--;
			CheckAndFree(handle->references[i]);
		}
	}
	mainHandle = NULL;
	kfree(handle->references);
	kfree(handle);
	
	printk(KERN_ERR"FD released PID:%d\n",current->pid);
	return 0;
}
int AllocateCMA(struct v3d2Handle *handle, unsigned int size,struct MemoryReference **refout, V3dMemoryHandle *out) {
	int i;
	struct MemoryReference *ref;
	for (i=0; i<handle->maxrefs; i++) if (handle->references[i] == 0) break;
	if (i == handle->maxrefs) {
		return -ENOMEM;
	}
	ref = kmalloc(sizeof(struct MemoryReference),GFP_KERNEL);
	if (!ref) return -ENOMEM;
	ref->size = size;
	ref->virt = dma_alloc_coherent(NULL,size,&ref->physical,GFP_KERNEL);
	if (!ref->virt) {
		kfree(ref);
		return -ENOMEM;
	}
	ref->mmap_count = 1;
	*refout = ref;
	printk(KERN_ERR"allocated %dKB at phys:%x virt:%p\n",size/1024,ref->physical,ref->virt);
	
	handle->references[i] = ref;
	*out = i;
	return 0;
}
static int qpu_enable(unsigned int enable) {
	int i = 1,ret;
	unsigned int message[32];
	message[i++] = 0; // request
	message[i++] = 0x30012; // set qpu enabled tag
	message[i++] = 4; // size of buffer
	message[i++] = 4; // size of data
	message[i++] = enable;
	message[i++] = 0; // end tag
	message[0] = i * sizeof(unsigned int);


	bcm_mailbox_property(message,i*sizeof(unsigned int));

	ret = message[5];

	return ret;
}
static int mem_lock(unsigned int handle) {
	int i = 1,ret;
	unsigned int message[32];
	message[i++] = 0; // request
	message[i++] = 0x3000d; // Lock memory
	message[i++] = 4; // size of buffer
	message[i++] = 4; // size of data
	message[i++] = handle;
	message[i++] = 0; // end tag
	message[0] = i * sizeof(unsigned int);

	bcm_mailbox_property(message,i*sizeof(unsigned int));
	
	ret = message[5];

	return ret;
}
static int mem_unlock(unsigned int handle) {
	int i = 1,ret;
	unsigned int message[32];
	message[i++] = 0; // request
	message[i++] = 0x3000e; // Unlock memory
	message[i++] = 4; // size of buffer
	message[i++] = 4; // size of data
	message[i++] = handle;
	message[i++] = 0; // end tag
	message[0] = i * sizeof(unsigned int);

	bcm_mailbox_property(message,i*sizeof(unsigned int));

	ret = message[5];

	return ret;
}
static int get_dispmanx_mem_handle(DISPMANX_RESOURCE_HANDLE_T resource) {
	int i = 1,ret;
	unsigned int message[32];
	message[i++] = 0; // request
	message[i++] = 0x30014; // Get Dispmanx Resource mem handle
	message[i++] = 8; // size of buffer
	message[i++] = 4; // size of data
	message[i++] = resource;
	message[i++] = 0; // filler
	message[i++] = 0; // end tag
	message[0] = i * sizeof(unsigned int);


	bcm_mailbox_property(message,i*sizeof(unsigned int));

	ret = message[6];

	return ret;
}
int compileJob(struct v3d2Handle *handle, Job *job, void *dest) {
	// dest is a temp buffer for the control list
	//printk(KERN_ERR"control list at %p %d\n",job->code,job->size);
	if (copy_from_user(dest,(const void __user *)job->code,job->size) != 0) {
		printk(KERN_ERR"fault\n");
		return -EFAULT;
	}
	//printk(KERN_ERR"kernvert:%p\n",dest);
	// the unlinked control list is now in this pointer
	// FIXME, this should scan over the list and link all memory addresses
	return 0;
}
static long v3d2_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	struct v3d2Handle *handle = filp->private_data;
	long ret = 0;
	V3dMemoryHandle hnd;
	struct mem_alloc_request request;
	struct MemoryReference *ref;

	printk(KERN_ERR"ioctl %x from PID:%d\n",cmd,current->pid);
	switch (cmd) {
	case V3D2_MEM_ALLOC:
		if (copy_from_user(&request,(const void __user *)arg,sizeof(request)) != 0) {
			ret = -EFAULT;
			break;
		}
		ret = AllocateCMA(handle,request.size,&ref,&request.handle);
		if (ret) break;
		request.physicalAddress = ref->physical;
		if (copy_to_user((void __user *)arg,&request,sizeof(request)) != 0) {
			ret = -EFAULT;
			break;
		}
		break;
	case V3D2_MEM_FREE:
		if (copy_from_user(&hnd,(const void __user *)arg,sizeof(hnd)) != 0) {
			ret = -EFAULT;
			break;
		}
		if (!handle->references[hnd]) {
			ret = -EINVAL;
			break;
		}
		if (handle->active == handle->references[hnd]) handle->active = 0;
		handle->references[hnd]->mmap_count--;
		CheckAndFree(handle->references[hnd]);
		handle->references[hnd] = 0;
		break;
	/*case V3D2_QPU_ENABLE:
		if ((v3dio[V3D_IDENT0] == 0x02443356) && (arg == 1)) {
			printk(KERN_ERR"qpu already online\n");
		} else {
			ret = qpu_enable(arg);
			if (v3dio[V3D_IDENT0] != 0x02443356) {
				printk(KERN_ERR"cant find magic number in v3d registers\n");
				ret = -EINVAL;
			}
		}
		break;*/
	case V3D2_MEM_SELECT:
		if (copy_from_user(&hnd,(const void __user *)arg,sizeof(hnd)) != 0) {
			ret = -EFAULT;
			break;
		}
		if (!handle->references[hnd]) {
			ret = -EINVAL;
			break;
		}
		handle->active = handle->references[hnd];
		break;
	case V3D2_COMPILE_CL:
		count = 0; // FIXME, remove this later?
		if (handle->activeJob) {
			printk(KERN_ERR"a job is already active\n");
			ret = -EBUSY;
			break;
		}
		handle->activeJob = kmalloc(sizeof(JobCompileRequest),GFP_KERNEL);
		if (copy_from_user(handle->activeJob,(const void __user *)arg,sizeof(JobCompileRequest)) != 0) {
			ret = -EFAULT;
			break;
		}
		if (handle->activeBinner) {
			printk(KERN_ERR"binner in use\n");
			ret = -EBUSY;
			break;
			// FIXME, memleak
		}
		if (handle->activeJob->outputType == opDispmanx) {
			// FIXME, find control tag 133, insert the raw addr, and unlock once frame is rendered
			int dispmanhandle = get_dispmanx_mem_handle(handle->activeJob->output.resource);
			uint32_t dispmanraw = mem_lock(dispmanhandle);
			printk(KERN_ERR"dispmanx buffer at %d %x\n",dispmanhandle,dispmanraw);
			mem_unlock(dispmanhandle);
		}
		handle->activeBinner = kmalloc(handle->activeJob->binner.size,GFP_KERNEL);
		handle->binnerSize = handle->activeJob->binner.size;
		ret = compileJob(handle,&handle->activeJob->binner,handle->activeBinner);
		handle->status.binnerFinished = 0;
		handle->status.rendererFinished = 0;
		handle->status.jobid = handle->activeJob->jobid;
			// FIXME, memleak
		if (ret) break;
		if (handle->activeJob->renderer.run == 0) handle->rs = notEnabled;
		else handle->rs = compiling;
		if (handle->activeJob->binner.run) {
			int status;
			uint32_t rawbinner = virt_to_phys(handle->activeBinner);
			//printk(KERN_ERR"starting binner on thread 0, old CS:%x, new CS:%x\n",v3dio[V3D_CT0CS],rawbinner);
			handle->bs = running;
			v3dio[V3D_CT0CS] = 8000;
			wmb();
			__asm volatile ("mcrr p15, #0, %[end], %[start], c12" : : [end] "r" (handle->activeBinner+handle->binnerSize), [start] "r" (handle->activeBinner) );
			v3dio[V3D_CT0CA] = rawbinner;
			wmb();
			v3dio[V3D_CT0EA] = rawbinner + handle->activeJob->binner.size;
			status = v3dio[V3D_CT0CS];
			if (status != 0x20) printk(KERN_ERR"binner started, status %x\n",status);
		}
		handle->activeRenderer = kmalloc(handle->activeJob->renderer.size,GFP_KERNEL);
		ret = compileJob(handle,&handle->activeJob->renderer,handle->activeRenderer);
			// FIXME, memleak
		if (ret) break;
		if (handle->activeJob->renderer.run) {
			unsigned long flags;
			// since this disables interupts, and the bcm2708 only has 1 arm core, the interupt doesnt have to lock it
			spin_lock_irqsave(&handle->lock,flags);
			if (handle->bs == finished) {
				// binner finished while linking
				uint32_t rawrenderer = virt_to_phys(mainHandle->activeRenderer);
				printk(KERN_ERR"starting renderer thread from ioctl %x + %d\n",rawrenderer,mainHandle->activeJob->renderer.size);
				mainHandle->rs = running;
				__asm volatile ("mcrr p15, #0, %[end], %[start], c12" : : [end] "r" (handle->activeRenderer+handle->rendererSize), [start] "r" (handle->activeRenderer) );
				v3dio[V3D_CT1CA] = rawrenderer;
				v3dio[V3D_CT1EA] = rawrenderer + mainHandle->activeJob->renderer.size;
			} else {
				handle->rs = waiting;
				//printk(KERN_ERR"queued renderer\n");
				handle->rendererSize = mainHandle->activeJob->renderer.size;
			}
			spin_unlock_irqrestore(&handle->lock,flags);
		}
		if (copy_to_user((void __user *)arg,handle->activeJob,sizeof(JobCompileRequest)) != 0) {
			ret = -EFAULT;
			// FIXME, memleak
			break;
		}
		kfree(mainHandle->activeJob);
		mainHandle->activeJob = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}
void v3d2_vma_open(struct vm_area_struct *vma) {
	struct MemoryReference *ref = vma->vm_private_data;
	ref->mmap_count++;
	//printk(KERN_NOTICE "V3D2 VMA open, virt %lx, phys %lx\n\n",vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}
void v3d2_vma_close(struct vm_area_struct *vma) {
	struct MemoryReference *ref = vma->vm_private_data;
	ref->mmap_count--;
	//printk(KERN_NOTICE "V3D2 VMA close.\n");
	CheckAndFree(ref);
}
static struct vm_operations_struct v3d2_remap_vm_ops = {
	.open = v3d2_vma_open,
	.close = v3d2_vma_close
};
// http://www.makelinux.net/ldd3/chp-15-sect-2
int v3d2_mmap(struct file *filp, struct vm_area_struct *vma) {
	struct v3d2Handle *handle = filp->private_data;
	if (!handle->active) {
		printk(KERN_ERR" no object selected\n");
		return -EINVAL;
	}

	// is this needed?
	vma->vm_pgoff = handle->active->physical >> PAGE_SHIFT;

	// FIXME, round up to the nearest whole page
	if (((vma->vm_end - vma->vm_start)>>PAGE_SHIFT) > ((handle->active->size>>PAGE_SHIFT)+1)) {
		printk(KERN_ERR"size greater then current object %d pages vs %d pages\n",(vma->vm_end - vma->vm_start)>>PAGE_SHIFT,(handle->active->size>>PAGE_SHIFT)+1);
		return -EINVAL;
	}
	if (remap_pfn_range(vma, vma->vm_start, handle->active->physical >> PAGE_SHIFT, vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		printk(KERN_ERR" remap failed\n");
		return -EAGAIN;
	}
	vma->vm_private_data = handle->active;
	vma->vm_ops = &v3d2_remap_vm_ops;
	v3d2_vma_open(vma);
	return 0;
}
ssize_t v3d2_read(struct file *filp,char __user *buffer, size_t size, loff_t *f_pos) {
	int ret,bytesToCopy;
	struct v3d2Handle *handle = filp->private_data;
	int boffset = 0;
	int roffset = 0;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&handle->lock,flags);
	if (handle->rs == idle) {
		spin_unlock_irqrestore(&handle->lock,flags);
		return -EWOULDBLOCK;
	}
	printk(KERN_ERR" pre b/r state %s %s\n",states[handle->bs],states[handle->rs]);
	if (handle->bs == running) {
		printk(KERN_ERR"binner CS:%x\n",v3dio[V3D_CT0CS]);
	}
	spin_unlock_irqrestore(&handle->lock,flags);
	
	//if (
	i = wait_event_interruptible_timeout(handle->queue,handle->rs == finished,HZ*10);
	// this returns the time remaining, not an error
	//) {
	//	printk(KERN_ERR"interrupted, status %d\n",i);
	//	return -ERESTARTSYS;
	//}
	printk(KERN_ERR"post b/r state %s %s\n",states[handle->bs],states[handle->rs]);
	handle->rs = idle;
	printk(KERN_ERR"woke up\n");
	
	if (handle->activeBinner) boffset = virt_to_phys(handle->activeBinner);
	if (handle->activeRenderer) roffset = virt_to_phys(handle->activeRenderer);
	
	printk(KERN_ERR"  CS CA EA\n");
	printk(KERN_ERR"%3x %4x %4x\n",v3dio[V3D_CT0CS],v3dio[V3D_CT0CA] - boffset,v3dio[V3D_CT0EA]-boffset);
	printk(KERN_ERR"%3x %4x %4x\n",v3dio[V3D_CT1CS],v3dio[V3D_CT1CA] - roffset,v3dio[V3D_CT1EA]-roffset);
	printk(KERN_ERR"%3x %4x %4x\n",v3dio[V3D_CT1CS],v3dio[V3D_CT1CA],v3dio[V3D_CT1EA]);
	if (handle->bs == running) {
		uint8_t *temp = handle->activeBinner;
		int pointer = v3dio[V3D_CT0CA] - boffset;
		printk(KERN_ERR" binner virt addr %p == ",temp);
		for (i=0; i<handle->binnerSize; i++) {
			if (pointer == i) printk("[%03d] ",temp[i]);
			else printk("%03d ",temp[i]);
		}
		printk("\n");
	}
	if (handle->rs == running) {
		uint8_t *temp = handle->activeRenderer;
		int pointer = v3dio[V3D_CT0CA] - roffset;
		printk(KERN_ERR" binner virt addr %p == ",temp);
		for (i=0; i<handle->rendererSize; i++) {
			if (pointer == i) printk("[%03d] ",temp[i]);
			else printk("%03d ",temp[i]);
		}
		printk("\n");
	}
	
	if (size > sizeof(JobStatusPacket)) bytesToCopy = sizeof(JobStatusPacket);
	else bytesToCopy = size;
	ret = bytesToCopy;
	if (copy_to_user(buffer,&handle->status,bytesToCopy)) {
		return -EFAULT;
	}
	*f_pos += bytesToCopy;
	return ret;
}
static struct file_operations v3d2_fops = {
	.owner		= THIS_MODULE,
	.open = v3d2_open,
	.read 		= v3d2_read,
	.release = v3d2_release,
	.unlocked_ioctl	= v3d2_ioctl,
	.mmap		= v3d2_mmap
};
static int __init v3d2_init(void) {
	int err = 0;
	
	err = alloc_chrdev_region(&characterDevice,0,1,"v3d2");
	if (err) goto fail;
	
	v3d2_cdev = cdev_alloc();
	if (!v3d2_cdev) goto fail2;
	
	v3d2_cdev->owner = THIS_MODULE;
	v3d2_cdev->ops = &v3d2_fops;
	
	err = cdev_add(v3d2_cdev,characterDevice,1);
	if (err) goto fail3;

	v3d2_class = class_create(THIS_MODULE,"v3d2");
	if (IS_ERR(v3d2_class)) goto fail3;

	device_create(v3d2_class,NULL,characterDevice,NULL,"v3d2");

	printk(KERN_INFO"v3d loaded\n");
	return err;
fail3:
	cdev_del(v3d2_cdev);
fail2:
	unregister_chrdev_region(characterDevice,1);
fail:
	return err;
}
static void __exit v3d2_exit(void) {
	device_destroy(v3d2_class,characterDevice);
	class_destroy(v3d2_class);
	if (characterDevice) unregister_chrdev_region(characterDevice,1);
	if (v3d2_cdev) cdev_del(v3d2_cdev);
	printk(KERN_INFO"v3d unloaded\n");
}
module_init(v3d2_init);
module_exit(v3d2_exit);
MODULE_LICENSE("GPL");
