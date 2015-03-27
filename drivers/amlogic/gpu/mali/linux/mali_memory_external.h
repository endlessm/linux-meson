#ifndef __MALI_MEMORY_EXTERNAL_H__
#define __MALI_MEMORY_EXTERNAL_H__

#ifdef __cplusplus
extern "C" {
#endif

_mali_osk_errcode_t mali_memory_bind_ext_mem(mali_mem_allocation *alloc,
		mali_mem_backend *mem_backend,
		u32 phys_addr,
		u32 flag);

void mali_memory_unbind_ext_mem(mali_mem_backend *mem_backend);

void mali_mem_external_release(mali_mem_backend *mem_backend);



#ifdef __cplusplus
}
#endif

#endif
