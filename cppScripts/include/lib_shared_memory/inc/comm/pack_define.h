#ifdef SR_WIN_NT_SUPPORT
#define SRCS_DEF_PACKED
#else
#define SRCS_DEF_PACKED __attribute__ ((packed))  
#endif