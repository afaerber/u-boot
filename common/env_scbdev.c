#include <common.h>
#include <command.h>
#include <environment.h>
#include <linux/stddef.h>
#include <malloc.h>
#include <search.h>
#include <errno.h>
#include <linux/mtd/mtd.h>

DECLARE_GLOBAL_DATA_PTR;

struct mtd_info *mtd;
env_t *env_ptr;
char *env_name_spec = "HSSPI NOR";

extern int scb_mtd_init(void);
extern struct mtd_info *mtd_table[MAX_MTD_DEVICES];

static int init_scb_dev(void)
{
	int err;
	int i;

	for (i = 0; i < MAX_MTD_DEVICES; i++) {
		mtd_table[i] = NULL;
	}

	err = scb_mtd_init();
	if(err) {
		printf("scb_mtd_init fail\n");
		goto fail;
	}

	/* find the mtd device */
	mtd = get_mtd_device_nm("scb_stroage");
	if(mtd == NULL) {
		printf("fail to find scb device\n");
		goto fail;
	}

	return 0;

fail:

	return -1;
}

static int read_env(struct mtd_info *mtd, void *buf)
{
	size_t len;
	int ret;
	if(mtd == NULL)
		return -1;
	ret =  mtd->read(mtd, 0, CONFIG_ENV_SIZE, &len, buf);
	return ret;
}

int env_init(void)
{
	/* let env_relocate_spec get the environment */
	gd->env_addr = (ulong)&default_environment[0];
	gd->env_valid = 1;
	
	return 0;
}

#ifdef CONFIG_CMD_SAVEENV
int saveenv(void)
{
	size_t len;
	env_t env_new;
	char *res = (char *)&env_new.data;

	len = hexport_r(&env_htab, '\0', &res, ENV_SIZE, 0, NULL);
	env_new.crc = crc32(0, env_new.data, ENV_SIZE);

	if(mtd == NULL)
		   return -1;

    struct erase_info instr;

    instr.addr = 0;
    instr.len = CONFIG_ENV_SIZE;
	instr.callback = NULL;

	if(!mtd->erase(mtd, &instr)) {
		return mtd->write(mtd, 0, CONFIG_ENV_SIZE, &len, (unsigned char *)&env_new);
	}
	
	return -1;
}
#endif /* CMD_SAVEENV */

void env_relocate_spec(void)
{
	int err;
	ALLOC_CACHE_ALIGN_BUFFER(char, buf, CONFIG_ENV_SIZE);
	/* read environment from scb block dev */
	err = init_scb_dev();
	if(!err) {
		err = read_env(mtd, (void *)buf);
	}

	if(err)
		memcpy(buf, &default_environment[0], CONFIG_ENV_SIZE);

	env_import((char *)buf, 1);
}
