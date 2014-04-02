#ifndef V3D2_IOCTL_H
#define V3D2_IOCTL_H
typedef uint16_t V3dMemoryHandle;
struct mem_alloc_request {
	unsigned int size;
//	unsigned int align;
	unsigned int flags;
	V3dMemoryHandle handle;
	unsigned int physicalAddress;
};
typedef struct ObjectReference {
	// currently, this is a raw byte offset
	// in future, it will be a variable offset, once the decompiler is done
	uint32_t location; // where in the control list it goes
	V3dMemoryHandle handle;
	uint32_t offset;
} ObjectReference;
typedef struct Job {
	uint8_t *code;
	uint32_t size;
	//ObjectReference *inputs;
	//uint32_t inputCount;
	//V3dMemoryHandle handle; // in/out, overwrite it, -1 to create
	uint8_t run;
} Job;
enum OutputType { opDispmanx, opMemoryHandle };
typedef struct JobCompileRequest {
	int jobid;
	Job binner;
	Job renderer;
	Job *uniforms;
	uint16_t uniformCount;
	enum OutputType outputType;
	union {
		DISPMANX_RESOURCE_HANDLE_T resource;
		V3dMemoryHandle handle;
	} output;
} JobCompileRequest;
typedef struct JobStatusPacket {
	int jobid;
	bool binnerFinished;
	bool rendererFinished;
} JobStatusPacket;

// FIXME, should i allocate this somewhere?
#define V3D2_IOC_MAGIC 0xda

#define V3D2_MEM_ALLOC			_IO(V3D2_IOC_MAGIC,	0) // struct mem_alloc_request*
#define V3D2_MEM_FREE			_IO(V3D2_IOC_MAGIC,	1) // V3dMemoryHandle*
#define V3D2_MEM_DEMO			_IO(V3D2_IOC_MAGIC,	2)
//#define V3D2_QPU_ENABLE			_IO(V3D2_IOC_MAGIC,	3) // int, 1==on, 0==off
#define V3D2_MEM_SELECT			_IO(V3D2_IOC_MAGIC,	4) // V3dMemoryHandle*
#define V3D2_COMPILE_CL			_IO(V3D2_IOC_MAGIC,	5) // JobCompileRequest*
#endif
