
#include <config.h>
#include <malloc.h>
#include <common.h>
#include <console.h>
#include <g_dnl.h>

#include <linux/err.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <ram_storage.h>

#include <asm/unaligned.h>
#include <linux/usb/gadget.h>
#include <linux/usb/gadget.h>
#include <linux/usb/composite.h>
#include <usb/lin_gadget_compat.h>
#include <g_dnl.h>

#define KERNEL_START_ADDR       0x17000
#define KERNEL_SIZE     0x71000

#define SPI_FLASH_CMD(file_addr)        \
        "sf probe 0 0 &&  \
        sf erase 0x17000 0x71000 && \
        sf write ##file_addr 0x17000 0x71000 "

#define DEBUG_rs 0
#define DBG_XFER_DATA   0
#define DEBUG_COMPLETE_HANDLER	0

#if DEBUG_rs
	#define dbg(x...) printf(x)
#else
	#define dbg(x...) do {} while (0)
#endif

#if DEBUG_COMPLETE_HANDLER
	#define dbg_hdlr(x...) printf(x)
#else
	#define dbg_hdlr(x...) do {} while (0)
#endif


#define OUT_XFER	0
#define IN_XFER	0x80
#define XFER_INIT_STAGE	0
#define XFER_INIT_STAGE_BYTE	8
#define XFER_DATA_STAGE	1
#define RAM_USB_STORAGE	"RAM USB Storage"
#define TEST_PAGE_SIZE          0x4000
#define BULK_PACKAGE_LENGTH     0x200
#define HID_RX_BUF              64
#define TIMEOUT                 3000

#define BUF_SZ_512B	512
#define BUF_SZ_5MB	0x500000
#define BUF_SZ_16MB	0x1000000
#define SPI_FLAS_CMD(file_addr) "sf probe 0 && " \
			 "sf erase 0x170000 0x710000 &&" \
			 "sf write file_addr 0x170000 0x710000 "

#define __get_bulk_ep_length(a) \
        ((a <= BUF_SZ_512B) ? a : BUF_SZ_512B)

char file_data_buffer[BUF_SZ_5MB];
static struct rs_drv_data *drv_data;

struct usb_rs_dev {
	struct rs_drv_data *drv_data;
	struct usb_gadget *gad;
	struct usb_function fun;
	struct usb_ep *in_ep, *out_ep;
	struct usb_request *out_req, *in_req;
	char *blk_in_buf, *blk_out_buf;
	int config_done;
	int data_xfer_done;
	spinlock_t lck;
	void *xfer_data;
	int expected_bytes;
};

struct rs_drv_data {
	struct usb_gadget *gad;
	struct usb_function *fun;
	struct usb_rs_dev *ram_dev, *hid_dev;
	int file_xfer_done;
};

/** interface descriptor */
static struct usb_interface_descriptor intf_desc = {
        .bLength = sizeof intf_desc,
        .bDescriptorType = USB_DT_INTERFACE,
        .bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR_SPEC,
        .bInterfaceSubClass = USB_SUBCLASS_VENDOR_SPEC,
        .bInterfaceProtocol = 0xff,
        .iInterface = 5,
};

/** full speed ep in discriptor */
static struct usb_endpoint_descriptor full_spd_bulk_in_desc = {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = USB_DIR_IN,
        .bmAttributes = USB_ENDPOINT_XFER_BULK,
};

/** full speed ep out discriptor */
static struct usb_endpoint_descriptor full_spd_bulk_out_desc = {
        .bLength =              USB_DT_ENDPOINT_SIZE,
        .bDescriptorType =      USB_DT_ENDPOINT,
        .bEndpointAddress =     USB_DIR_OUT,
        .bmAttributes =         USB_ENDPOINT_XFER_BULK,
};

/** high speed ep in discriptor */
static struct usb_endpoint_descriptor high_spd_bulk_in_desc = {
        .bLength =              USB_DT_ENDPOINT_SIZE,
        .bDescriptorType =      USB_DT_ENDPOINT,
        .bmAttributes =         USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize =       cpu_to_le16(512),
};

/** high speed ep out discriptor */
static struct usb_endpoint_descriptor high_spd_bulk_out_desc = {
        .bLength =              USB_DT_ENDPOINT_SIZE,
        .bDescriptorType =      USB_DT_ENDPOINT,
        .bmAttributes =         USB_ENDPOINT_XFER_BULK,
        .wMaxPacketSize =       cpu_to_le16(512),
        .bInterval =            1,      /* NAK every 1 uframe */
};

static struct usb_descriptor_header *high_spd_function[] = {
        (struct usb_descriptor_header *) &intf_desc,
        (struct usb_descriptor_header *) &high_spd_bulk_in_desc,
        (struct usb_descriptor_header *) &high_spd_bulk_out_desc,
        NULL,
};

static struct usb_descriptor_header *full_spd_function[] = {
        (struct usb_descriptor_header *) &intf_desc,
        (struct usb_descriptor_header *) &full_spd_bulk_in_desc,
        (struct usb_descriptor_header *) &full_spd_bulk_out_desc,
        NULL,
};

/* Static strings, in UTF-8 */
static const char usb_string_interface[] = "RAM Mass Storage";
static const char usb_string_serial[] = "5156";
static const char usb_string_config[] = "";

enum {
        USB_STRING_MANUFACTURER = 1,
        USB_STRING_PRODUCT,
        USB_STRING_SERIAL,
        USB_STRING_CONFIG,
        USB_STRING_INTERFACE = 5
};

static struct usb_string rs_strings[] = {
        {USB_STRING_MANUFACTURER, usb_string_manufacturer},
        {USB_STRING_PRODUCT, usb_string_product},
        {USB_STRING_SERIAL, usb_string_serial},
        {USB_STRING_CONFIG, usb_string_config},
        {USB_STRING_INTERFACE, usb_string_interface},
        {}
};

static struct usb_gadget_strings rs_gad_strings = {
        .language       = 0x0409,               /* en-us */
        .strings        = rs_strings,
};

static struct usb_gadget_strings *rs_strings_array[] = {
        &rs_gad_strings,
        NULL,
};

void print_test_cmd(test_cmd *cmd)
{
	printf("%s : id = %x\n", __func__, cmd->id);
	printf("%s : status = %x\n", __func__, cmd->status);
	printf("%s : length = %x\n", __func__, cmd->len);
}

void print_ep(const struct usb_endpoint_descriptor *ep_desc)
{
	printf("%s : bLength %x\n", __func__, ep_desc->bLength);
	printf("%s : bDescriptorType %x\n", __func__, ep_desc->bDescriptorType);
	printf("%s : bEndpointAddress %x\n", __func__, ep_desc->bEndpointAddress);
	printf("%s : bmAttributes %x\n", __func__, ep_desc->bmAttributes);
	printf("%s : wMaxPacketSize %x\n", __func__, ep_desc->wMaxPacketSize);
	printf("%s : bRefresh %d\n", __func__, ep_desc->bRefresh);
	printf("%s : bSynchAddress %x\n", __func__, ep_desc->bSynchAddress);
}

void print_hex(const char *buf, int size)
{
	int i = 0;
	printf("%s : data :", __func__);
	while(i < size)
		printf("%x ", buf[i++]);
	printf("\n");
}

inline int rs_read(void *buf, uint32_t len)
{
	return rs_xfer(OUT_XFER, buf, len);
}

inline int rs_write(void *buf, uint32_t len)
{
	return rs_xfer(IN_XFER, buf, len);
}

static void rs_gad_disable(struct usb_function *func)
{
	struct usb_rs_dev *ram_dev =
                        container_of(func, struct usb_rs_dev, fun);
	dbg("%s : ram_dev = %p, in_driver data = %p\n", __func__,
		ram_dev, ram_dev->in_ep->driver_data);

	if (ram_dev->in_ep->driver_data) {
		dbg("%s : in free buf %p\n", __func__, ram_dev->in_req->buf);
		if (ram_dev->in_req->buf)
			free(ram_dev->in_req->buf);
		dbg("%s : freeing ep request\n", __func__);
		usb_ep_free_request(ram_dev->in_ep, ram_dev->in_req);
		dbg("%s : disablng ep\n", __func__);
                usb_ep_disable(ram_dev->in_ep);
                ram_dev->in_ep->driver_data = NULL;
        }

	dbg("%s : ram_dev = %p, out_driver data = %p\n", __func__,
		ram_dev, ram_dev->out_ep->driver_data);
        if (ram_dev->out_ep->driver_data) {
		dbg("%s : out free buf %p\n", __func__, ram_dev->in_req->buf);
		if (ram_dev->out_req->buf)
	                free(ram_dev->out_req->buf);
                ram_dev->out_req->buf = NULL;
		dbg("%s : out free request %p\n", __func__, ram_dev->out_req->buf);
                usb_ep_free_request(ram_dev->out_ep, ram_dev->out_req);
		dbg("%s : disablng out ep\n", __func__);
                usb_ep_disable(ram_dev->out_ep);
                ram_dev->out_ep->driver_data = NULL;
        }
}

static inline struct
usb_endpoint_descriptor *get_ep_desc(struct usb_gadget *g,
			struct usb_endpoint_descriptor *hs,
        		struct usb_endpoint_descriptor *fs)
{
        if (gadget_is_dualspeed(g) && g->speed == USB_SPEED_HIGH)
                return hs;
        return fs;
}

static struct usb_request *rs_alloc_ep_req(struct usb_ep *ep,
			unsigned length)
{
        struct usb_request *req;

        req = usb_ep_alloc_request(ep, 0);
        if (!req)
                return req;

        req->length = length;
//	req->buf = kzalloc(length, GFP_KERNEL);
	req->buf = memalign(CONFIG_SYS_CACHELINE_SIZE, length);
        if (!req->buf) {
                usb_ep_free_request(ep, req);
                req = NULL;
        }

        return req;
}

static void rs_xfer_complete(struct usb_ep *ep,
			struct usb_request *req)
{
        struct usb_rs_dev *ram_dev = ep->driver_data;
        int status = req->status;
	static int data_offset = 0;
	unsigned long flags;
#if 0
        dbg_hdlr("%s : ep = %p, out_ep = %p, in_ep = %p, xfer_data = %p\n",
		__func__, ep, ram_dev->out_ep,
		ram_dev->in_ep, ram_dev->xfer_data);
	dbg_hdlr("%s : req = %p, out_req = %p, in_req = %p\n",
		__func__, req, ram_dev->out_req, ram_dev->in_req);
	dbg_hdlr("%s : length = %d,actual = %d, status = %d\n",
		__func__, req->length, req->actual, status);
#endif

#if DEBUG_COMPLETE_HANDLER
	dbg_hdlr("%s : printing buffer : status = %d\n", __func__, status);
	print_hex(req->buf, req->actual);
#endif
        switch (status) {
		case 0:
			break;

		case -ECONNABORTED:
		case -ECONNRESET:
		case -ESHUTDOWN:
		case -EREMOTEIO:
		case -EOVERFLOW:
			error("ERROR:%d", status);
			break;
        }

	memcpy(ram_dev->xfer_data + data_offset, req->buf, req->actual);
	ram_dev->expected_bytes -= req->actual;
	data_offset += req->actual;
	dbg_hdlr("%s : expected_bytes = %d\n",
		__func__, ram_dev->expected_bytes);
        if (!(ram_dev->expected_bytes)) {
                spin_lock_irqsave(ram_dev->lck, flags);
                ram_dev->data_xfer_done = 1;
                spin_unlock_irqrestore(drv_data->lck, flags);
		data_offset = 0;
        } else {
		dbg_hdlr("%s : resubmission of usb request\n", __func__);
                usb_ep_queue(ep, req, 0);
        }
	dbg_hdlr("%s : data_offset = %d\n", __func__, data_offset);
}

static struct usb_request *rs_bulk_start_ep(struct usb_ep *ep)
{
        struct usb_request *req;

        req = rs_alloc_ep_req(ep, BUF_SZ_512B);
        dbg("%s: ep:%p req:%p\n", __func__, ep, req);

        if (!req)
                return NULL;

        memset(req->buf, 0, req->length);
        req->complete = rs_xfer_complete;

        return req;
}

static int rs_blk_xfer_setup(struct usb_function *func)
{
        struct usb_composite_dev *comp_dev = func->config->cdev;
        struct usb_gadget *gadget = comp_dev->gadget;
	struct usb_rs_dev *ram_dev =
                        container_of(func, struct usb_rs_dev, fun);

        struct usb_endpoint_descriptor *ep_desc;
        struct usb_request *req;
        struct usb_ep *ep;
        int result;

        ep = ram_dev->in_ep;

	ep_desc = get_ep_desc(gadget, &high_spd_bulk_in_desc, &full_spd_bulk_in_desc);
        dbg("%s : ep bEndpointAddress: 0x%x\n", __func__, ep_desc->bEndpointAddress);
        dbg("%s : bulkin_ep bEndpointAddress: 0x%x\n", __func__, ep->desc->bEndpointAddress);

        result = usb_ep_enable(ep, ep_desc);
        if (result)
                goto exit;

        ep->driver_data = ram_dev;
        req = rs_bulk_start_ep(ep);
        if (!req) {
                usb_ep_disable(ep);
                result = -EIO;
                goto exit;
        }

        ram_dev->in_req = req;

        ep = ram_dev->out_ep;
	ep_desc = get_ep_desc(gadget, &high_spd_bulk_out_desc, &full_spd_bulk_out_desc);
        dbg("%s : bulkot_ep EndpointAddress: 0x%x\n", __func__, ep_desc->bEndpointAddress);
	dbg("%s : bulkout_ep bEndpointAddress: 0x%x\n", __func__, ep->desc->bEndpointAddress);

        result = usb_ep_enable(ep, ep_desc);
        if (result)
                goto exit;

        ep->driver_data = ram_dev;
        req = rs_bulk_start_ep(ep);
        if (!req) {
                usb_ep_disable(ep);
                result = -EIO;
                goto exit;
        }

        ram_dev->out_req = req;
        ep->driver_data = ram_dev;
 exit:
        return result;
}

static int rs_gad_set_alt(struct usb_function *func,
                                        unsigned interface, unsigned alt)
{
	struct usb_rs_dev *ram_dev =
                        container_of(func, struct usb_rs_dev, fun);
	int ret;

	dbg("%s : func_name  = %s, intf = %d, alt = %d\n",
		__func__, func->name, interface, alt);

	switch(interface) {
		case 0:
			dbg("%s : data transefer to ram\n", __func__);
			if ((ret = rs_blk_xfer_setup(func)) < 0 )
				error("%s : failed to altset\n", __func__);
			ram_dev->config_done = 1;
			break;
		case 1: /** for future use */
			dbg("%s : INTR interface\n", __func__);
	                break;
	}

	dbg("%s : calling set alt\n", __func__);
	return 0;
}

static int rs_gad_setup(struct usb_function *fun,
                                        const struct usb_ctrlrequest *cntrl_req)
{
	dbg("%s : calling set\n", __func__);
	return 0;
}

static void rs_gad_unbind(struct usb_configuration *config,
			 struct usb_function *func)
{
	struct usb_rs_dev *ram_dev =
                        container_of(func, struct usb_rs_dev, fun);
	dbg("%s : ram_dev->fun.descriptors = %p\n", __func__, ram_dev->fun.descriptors);
	if (ram_dev->fun.descriptors)
	        kfree(ram_dev->fun.descriptors);
	dbg("%s : ram_dev->fun.hs_descriptors = %p\n", __func__, ram_dev->fun.hs_descriptors);
	if (ram_dev->fun.hs_descriptors)
		kfree(ram_dev->fun.hs_descriptors);

	dbg("%s : unbind ram_dev->drv_data->ram_dev = %p, ram_dev = %p\n",
		__func__, ram_dev->drv_data->ram_dev, ram_dev);
        if (ram_dev->drv_data->ram_dev == ram_dev) {
		kfree(ram_dev);
	}
	dbg("%s : drv_data = %p\n", __func__, ram_dev);
	if (drv_data)
		kfree(drv_data);
}

struct usb_descriptor_header
	**usb_copy_ram_descriptors(struct usb_descriptor_header **src)
{
        struct usb_descriptor_header **tmp;
        unsigned bytes;
        unsigned n_desc;
        void *mem;
        struct usb_descriptor_header **ret;

        for (bytes = 0, n_desc = 0, tmp = src; *tmp; tmp++, n_desc++)
                bytes += (*tmp)->bLength;
        bytes += (n_desc + 1) * sizeof(*tmp);

	mem = memalign(CONFIG_SYS_CACHELINE_SIZE, bytes);
//	mem = kzalloc(bytes, GFP_KERNEL);
        if (!mem)
                return NULL;

   	tmp = mem;
        ret = mem;
        mem += (n_desc + 1) * sizeof(*tmp);
        while (*src) {
                memcpy(mem, *src, (*src)->bLength);
                *tmp = mem;
                tmp++;
                mem += (*src)->bLength;
                src++;
        }
        *tmp = NULL;

        return ret;
}

static int rs_gad_bind(struct usb_configuration *config,
			 struct usb_function *func)
{
	struct usb_rs_dev *ram_dev =
			container_of(func, struct usb_rs_dev, fun);
	struct rs_drv_data *drv_data = ram_dev->drv_data;
	struct usb_gadget *gad = config->cdev->gadget;
	struct usb_ep *ep;
	int intf, ret;

        intf = usb_interface_id(config, func);
	dbg("%s : ram_dev = %p\n", __func__, ram_dev);
	dbg("%s : intf = %d\n", __func__, intf);

	intf_desc.bInterfaceNumber = intf;
        intf_desc.bInterfaceClass = 0xff;
        intf_desc.bInterfaceSubClass = 0;
	intf_desc.bInterfaceProtocol = 0xff;

	ep = usb_ep_autoconfig(gad, &full_spd_bulk_in_desc);
        dbg("%s : usb_ep_autoconfig(bulk_in), ep = %p\n", __func__, ep);
        if (!ep)
                goto ep_autoconf_fail;
	ep->driver_data = drv_data;  /* claim the endpoint */
        ram_dev->in_ep = ep;

#ifdef DEBUG
	dbg("%s : bulkin_ep =>\n", __func__);
        print_ep(&full_spd_bulk_in_desc);
#endif
	ep = usb_ep_autoconfig(gad, &full_spd_bulk_out_desc);
        dbg("%s : usb_ep_autoconfig(bulk_out), ep = %p\n", __func__, ep);
        if (!ep)
                goto ep_autoconf_fail;
#ifdef DEBUG
	dbg("%s : only ep =>\n", __func__);
	print_ep(&full_spd_bulk_out_desc);
#endif
        ep->driver_data = drv_data;  /* claim the endpoint */
        ram_dev->out_ep = ep;

	func->descriptors = usb_copy_ram_descriptors(full_spd_function);
        dbg("%s : usb_copy_descriptors, descriptors = %p\n",
		__func__, func->descriptors);
	if(!func->descriptors)
		return -ENOMEM;

	ret = gadget_is_dualspeed(gad);
	dbg("%s : gadget_is_dualspeed ret = %d\n", __func__, ret);
	if (ret) {
                high_spd_bulk_in_desc.bEndpointAddress =
                        full_spd_bulk_in_desc.bEndpointAddress;
                dbg("%s : in bEndpointAddress = %x\n",
			__func__, full_spd_bulk_in_desc.bEndpointAddress);
                high_spd_bulk_out_desc.bEndpointAddress =
                        full_spd_bulk_out_desc.bEndpointAddress;
                dbg("%s : out bEndpointAddress = %x\n",
			__func__, high_spd_bulk_out_desc.bEndpointAddress);
                func->hs_descriptors = usb_copy_ram_descriptors(high_spd_function);
                if (unlikely(!func->hs_descriptors)) {
                        free(func->descriptors);
                        return -ENOMEM;
                }
        }

	return 0;

ep_autoconf_fail:
        dbg("%s : unable to autoconfigure all endpoints\n", __func__);
        return -ENOTSUPP;
}

static int rs_gad_add_fun(struct usb_configuration *config)
{
	struct usb_rs_dev *ram_dev = drv_data->ram_dev;
	int ret;
	dbg("%s : ram_dev = %p\n", __func__, ram_dev);
	dbg("%s : adding function\n", __func__);
	ram_dev->fun.name = RAM_USB_STORAGE;
	ram_dev->fun.strings = rs_strings_array;
	ram_dev->fun.bind = rs_gad_bind;
	ram_dev->fun.unbind = rs_gad_unbind;
	ram_dev->fun.setup = rs_gad_setup;
	ram_dev->fun.set_alt = rs_gad_set_alt;
	ram_dev->fun.disable = rs_gad_disable;

	drv_data->fun = &ram_dev->fun;
	drv_data->ram_dev = ram_dev;

	ret = usb_add_function(config, &ram_dev->fun);
	dbg("%s : usb_add_function = %d\n", __func__, ret);
        if (ret)
        	kfree(ram_dev);

	return ret;
}

static int rs_gad_init(struct usb_composite_dev *comp_dev,
		struct usb_configuration *usb_config)
{
	struct usb_gadget *gad = comp_dev->gadget;
	struct usb_rs_dev *ram_dev;
	int ret;

	drv_data = kzalloc(sizeof(struct rs_drv_data), GFP_KERNEL);
	dbg("%s : drv_data = %p\n", __func__, drv_data);
	if (!drv_data) {
		dbg("%s : falied to alloc drv drv_data\n", __func__);
		return -ENOMEM;
	}

	ram_dev = kzalloc(sizeof(struct usb_rs_dev), GFP_KERNEL);
	dbg("%s : ram_dev = %p\n", __func__, ram_dev);
	if (!ram_dev) {
		dbg("%s : falied to alloc ram dev mem\n", __func__);
                return -ENOMEM;
	}

	drv_data->gad = gad;
	ram_dev->drv_data = drv_data;
	drv_data->ram_dev = ram_dev;
	spin_lock_init(ram_dev->lck);

        if (rs_strings[USB_STRING_INTERFACE].id == 0) {
                if((ret = usb_string_id(comp_dev)) < 0)
                        return ret;
                rs_strings[USB_STRING_INTERFACE].id = ret;
                intf_desc.iInterface = ret;
        }

	return rs_gad_add_fun(usb_config);
}

int rs_gad_add(struct usb_configuration *usb_config)
{
	struct usb_composite_dev *comp_dev = usb_config->cdev;
	int ret = 0;

	dbg("%s : calling ram storage initing\n", __func__);
	if ((ret = rs_gad_init(comp_dev, usb_config)) < 0)
		dbg("%s : failed to init ram storage device ret = %d\n",
			__func__, ret);
	return ret;
}

int rs_xfer(int dir, void *data, int length)
{
        struct usb_rs_dev *ram_dev = drv_data->ram_dev;
        int status;
	struct usb_request *req;
	struct usb_ep *ep;

	dbg("%s : dir = %d, bytes of data to be written = %d\n",
		__func__, dir, length);
	if (dir == OUT_XFER) {
		req = ram_dev->out_req;
		ep = ram_dev->out_ep;
	} else {
		req = ram_dev->in_req;
                ep = ram_dev->in_ep;
		memcpy(req->buf, data, length);
	}
	req->length = __get_bulk_ep_length(length);
	ram_dev->expected_bytes = length;
	ram_dev->xfer_data = data;
#if  0
	dbg("%s : req = %p, ep = %p, ram_dev->xfer_data = %p\n", __func__, req, ep, ram_dev->xfer_data);
#endif
        do {
        	dbg("%s : req->length = %d\n", __func__, req->length);
#if 0
		dbg("%s : submitting test init xfer_cmd : dir = %d\n", __func__, dir);
		print_test_cmd(data);
#endif
                status = usb_ep_queue(ep, req, 0);
		dbg("%s : usb_ep_queue, ret = %d\n", __func__, status);
                if (status) {
                        error("kill %s :  resubmit %d bytes --> %d",
                                ep->name, req->length, status);
                        usb_ep_set_halt(ep);
                        return -EAGAIN;
                }

		dbg("%s : xfer_done = %d\n", __func__,
				ram_dev->data_xfer_done);

		while (!ram_dev->data_xfer_done)
			usb_gadget_handle_interrupts(0);

		if (ram_dev->data_xfer_done) {
			spin_lock(&ram_dev->lck);
			ram_dev->data_xfer_done = 0;
			spin_unlock(&ram_dev->lck);
#if 0
			dbg("%s printing recivd data\n", __func__);
			print_hex(data, length);
#endif
			break;
		}

		if (ctrlc())
			return -1;
        } while (0);

        return length;
}

static int rs_action_handler(test_cmd  xfer_cmd)
{
	int ret;
	char cmd_buf[128];
#if 0
	dbg("%s : printing xfer_cmd\n", __func__);
	print_test_cmd(&xfer_cmd);
#endif
	switch(xfer_cmd.id) {
		case 0x1:
			dbg("%s : cmd init command\n", __func__);
			xfer_cmd.status = 0x1;
			ret = rs_write(&xfer_cmd, sizeof(test_cmd));
			break;

		case 0x2:
			dbg("%s : firmware downloading \n", __func__);
			printf("File is being downloaded ...\n");
			xfer_cmd.id = 0x2;
			ret = rs_write(&xfer_cmd, sizeof(xfer_cmd));
			dbg("%s : handshaking for fdnl is done, ret = %d\n", __func__, ret);
			dbg("%s : xfer_cmd.len size = %d\n", __func__, xfer_cmd.len);
			memset(file_data_buffer, '\0', xfer_cmd.len);
			ret = rs_read(file_data_buffer, xfer_cmd.len);
#if DBG_XFER_DATA
			printf("%s :  dnloaded buf, ret = %d, ptr_buf = %p\n",
				__func__, ret, file_data_buffer);
			print_hex(file_data_buffer, xfer_cmd.len);
#endif
			drv_data->file_xfer_done = 1;
			dbg("%s file is recived and checking do_command\n", __func__);
			memset(cmd_buf, '\0', sizeof(cmd_buf));
			printf("Flashing file ...\n");
			sprintf(cmd_buf, "setenv rs_flash_file 'sf probe 0 && sf erase 0 0x470000 && sf write %p 0 0x470000'", file_data_buffer);
			dbg("%s : cmd_buf = %s\n", __func__, cmd_buf);

			run_command(cmd_buf, 0);
			run_command("saveenv && run rs_flash_file", 0);
			break;

		default:
			ret = -1;
			break;
	}

	return ret;
}

static int rs_data_xfer_handler(void)
{
	test_cmd xfer_cmd;
        int ret = 0;
	do {
		drv_data->file_xfer_done = 0;
		memset(&xfer_cmd, '\0', sizeof(test_cmd));
		ret = rs_read(&xfer_cmd, sizeof(test_cmd));
		dbg("%s : reciving test command ret = %d\n",
			__func__, ret);
#if 0
    		dbg("%s : printing xfer_cmd\n", __func__);
		print_test_cmd(&xfer_cmd);
#endif
		ret = rs_action_handler(xfer_cmd);
		dbg("%s : ret from rs_action_handler = %d, file_xfer_done = %d\n",
		__func__, ret, drv_data->file_xfer_done);

		if (ctrlc())
                        return -1;

	} while(ret > 0 && !drv_data->file_xfer_done);

	return ret;
}

int rs_init(void)
{
	struct usb_rs_dev *ram_dev = drv_data->ram_dev;
	while(!ram_dev->config_done)
		usb_gadget_handle_interrupts(0);

	return rs_data_xfer_handler();
}

DECLARE_GADGET_BIND_CALLBACK(rs, rs_gad_add);
