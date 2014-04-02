struct MemoryReference {
	void *virt;
	dma_addr_t physical;
	unsigned int size;
	unsigned int mmap_count; // includes the main references list, which isnt actually mmap
};

// must match the def in https://github.com/raspberrypi/userland/blob/master/interface/vmcs_host/vc_vchi_bufman.h
typedef uint32_t DISPMANX_RESOURCE_HANDLE_T;
