
#include <errno.h>
#include <common.h>
#include <command.h>
#include <console.h>
#include <g_dnl.h>
#include <part.h>
#include <usb.h>
#include <ram_storage.h>

#define DEBUG_rs  0
#define DBG_XFER_DATA   0

#if DEBUG_rs
        #define dbg(x...) printf(x)
#else
        #define dbg(x...) do {} while (0)
#endif

#define CONTROLLER_INDEX	0

/**
  init communication with test bulk app
  reciving test_cmd sent by bulk app
 */
#if 0
static int ram_stor_xfer_init(void)
{
	int ret;

	ret = ram_stor_xfer(OUT_XFER, XFER_INIT_STAGE);
        dbg("%s : xfer init recv, ret = %d\n", __func__, ret);
        if (ret) {
                error("fail : xfer init recive data %d\n", ret);
                return CMD_RET_FAILURE;
        }

        ret = ram_stor_xfer(IN_XFER, XFER_INIT_STAGE);
        dbg("%s : xfer init hanshaking ret = %d\n", __func__, ret);
        if (ret) {
                error("fail : xfer init handshaking %d\n", ret);
                return CMD_RET_FAILURE;
	}
	return ret;
}
#endif
int do_usb_ram_storage(cmd_tbl_t *cmdtp, int flag,
			       int argc, char * const argv[])
{
	int ret;
	int cable_ready_timeout __maybe_unused;
/*
	if (argc < 3)
		return CMD_RET_USAGE;

	if (argc >= 3) {
		usr_data.ram_start_addr = (unsigned int)(simple_strtoul(
                                	argv[1], NULL, 0));
		usr_data.ram_file_sz = (unsigned int)(simple_strtoul(
                                	argv[2], NULL, 0));
	}
*/
#if DEBUG
	dbg("%s : ram_start_addr = %x, ram_file_sz = %d\n", __func__,
		usr_data.ram_start_addr, usr_data.ram_file_sz);
#endif
	ret = board_usb_init(CONTROLLER_INDEX, USB_INIT_DEVICE);
        if (ret) {
                error("USB init failed: %d", ret);
                return CMD_RET_FAILURE;
        }

	ret = g_dnl_register("rs");
	dbg("%s : ret = %d\n", __func__, ret);
	if (ret) {
		error("g_dnl_register failed");
		ret = CMD_RET_FAILURE;
		goto cleanup_board;
	}


	/** enumeration */
	ret = rs_init();
	dbg("%s : calling ram_stor_init, ret = %d\n", __func__, ret);
	if (ret <  0){
		error("fail : RAM storage init %d\n", ret);
                ret = CMD_RET_FAILURE;
		goto cleanup_dnl_register;
	}
#if 0
	/**
          init communication with test  app
	  reciving test_cmd sent by bulk app
	*/
	ret = ram_stor_xfer_init();
	dbg("%s : calling ram_stor_init, ret = %d\n", __func__, ret);
        if (ret) {
                error("fail : RAM storage init %d\n", ret);
                ret = CMD_RET_FAILURE;
                goto cleanup_dnl_register;
        }
#endif
	printf("Flash is happened successful\n");

cleanup_dnl_register:
	g_dnl_unregister();
cleanup_board:
	board_usb_cleanup(CONTROLLER_INDEX, USB_INIT_DEVICE);
	return ret;
}

U_BOOT_CMD(rs, 4, 1, do_usb_ram_storage,
	"Use the RS {RAM Storage]",
	"<USB_controller> <ram_addr>  e.g. rs 0 <ram_addr>\n"
);
