#ifdef MB86S7X_SHM_FROM_SCB

#define SHM_OFFSET 0x800

#define cmd_from_scb ((void *)MB86S7X_SHM_FROM_SCB + SHM_OFFSET)
#define rsp_from_scb ((void *)(MB86S7X_SHM_FROM_SCB + SHM_OFFSET + 0x100))
#define cmd_to_scb ((void *)(MB86S7X_SHM_FROM_SCB + SHM_OFFSET + 0x200))
#define rsp_to_scb ((void *)(MB86S7X_SHM_FROM_SCB + SHM_OFFSET + 0x300))
#define brsel_from_scb ((void *)(MB86S7X_SHM_FROM_SCB + 0xEC0))

#define INTR_STAT_OFS  0
#define INTR_SET_OFS   8
#define INTR_CLR_OFS   0x10

#define MHU_SCFG       0x400

#define MB86S7X_SYS_FLASH_SIZE_MASK 0x00000003
#define MB86S7X_SYS_FLASH_SIZE_8MB  0x00000001
#define MB86S7X_SYS_FLASH_SIZE_2MB  0x00000002
#define MB86S7X_SYS_FLASH_SIZE_4MB  0x00000003

extern int mhu_send(u32 cmd);

extern u32 get_scb_version(void);
extern u32 get_sys_flash_size(void);
extern int set_power_state(u32 pd_index, u32 state);
extern int get_power_state(u32 pd_index);
extern int set_clk_state(u32 cntrlr, u32 domain, u32 port, u32 en);
extern int get_memory_layout(void);
extern int mhu_check_pcie_capability(void);
extern int mhu_check_video_out_capability(void);

#endif

