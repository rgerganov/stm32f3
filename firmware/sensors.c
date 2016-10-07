/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#define I2C_ACC_ADDR 0x19
#define I2C_MAG_ADDR 0x1E

#define ACC_CTRL_REG1_A 0x20
#define ACC_CTRL_REG4_A 0x23
#define ACC_STATUS_REG_A 0x27
#define ACC_OUT_X_L_A 0x28
#define ACC_OUT_X_H_A 0x29
#define ACC_OUT_Y_L_A 0x2A
#define ACC_OUT_Y_H_A 0x2B
#define ACC_OUT_Z_L_A 0x2C
#define ACC_OUT_Z_H_A 0x2D

#define MAG_CRA_REG_M 0x00
#define MAG_CRB_REG_M 0x01
#define MAG_MR_REG_M 0x02
#define MAG_OUT_X_H_M 0x03
#define MAG_OUT_X_L_M 0x04
#define MAG_OUT_Z_H_M 0x05
#define MAG_OUT_Z_L_M 0x06
#define MAG_OUT_Y_H_M 0x07
#define MAG_OUT_Y_L_M 0x08
#define MAG_SR_REG_M 0x09
#define TEMP_OUT_H_M 0x31
#define TEMP_OUT_L_M 0x32
#define MAG_GAIN_1_3 0x20
#define MAG_GAIN_1_9 0x40
#define MAG_GAIN_2_5 0x60
#define MAG_GAIN_4_0 0x80
#define MAG_GAIN_8_1 0xE0

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}, {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x82,
    .bmAttributes = USB_ENDPOINT_ATTR_BULK,
    .wMaxPacketSize = 64,
    .bInterval = 1,
}};

static const struct {
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header = {
        .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
        .bcdCDC = 0x0110,
    },
    .call_mgmt = {
        .bFunctionLength =
            sizeof(struct usb_cdc_call_management_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
        .bmCapabilities = 0,
        .bDataInterface = 1,
    },
    .acm = {
        .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_ACM,
        .bmCapabilities = 0,
    },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
     },
};

static const struct usb_interface_descriptor comm_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 1,
    .bInterfaceClass = USB_CLASS_CDC,
    .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
    .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
    .iInterface = 0,

    .endpoint = comm_endp,

    .extra = &cdcacm_functional_descriptors,
    .extralen = sizeof(cdcacm_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = comm_iface,
}, {
    .num_altsetting = 1,
    .altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static const char *usb_strings[] = {
    "Black Sphere Technologies",
    "CDC-ACM Demo",
    "DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch (req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
        /*
         * This Linux cdc_acm driver requires this to be implemented
         * even though it's optional in the CDC spec, and we don't
         * advertise it in the ACM functional descriptor.
         */
        char local_buf[10];
        struct usb_cdc_notification *notif = (void *)local_buf;

        /* We echo signals back to host as notification. */
        notif->bmRequestType = 0xA1;
        notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
        notif->wValue = 0;
        notif->wIndex = 0;
        notif->wLength = 2;
        local_buf[8] = req->wValue & 3;
        local_buf[9] = 0;
        // usbd_ep_write_packet(0x83, buf, 10);
        return 1;
        }
    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding))
            return 0;
        return 1;
    }
    return 0;
}

static void my_usb_print_int(usbd_device *usbd_dev, int32_t value)
{
    int8_t i;
    int8_t digits_ind = 0;
    char digits[25];
    int8_t buffer_ind = 0;
    char buffer[25];

    if (value < 0) {
        buffer[buffer_ind++] = '-';
        value = value * -1;
    }

    if (value == 0) {
        buffer[buffer_ind++] = '0';
    }

    while (value > 0) {
        digits[digits_ind++] = "0123456789"[value % 10];
        value /= 10;
    }

    for (i = digits_ind-1; i >= 0; i--) {
        buffer[buffer_ind++] = digits[i];
    }

    buffer[buffer_ind++] = '\r';
    buffer[buffer_ind++] = '\n';

    usbd_ep_write_packet(usbd_dev, 0x82, buffer, buffer_ind);
}

static void enable_accelerometer(void)
{
    // enable accelerometer, 100Hz
    uint8_t data[1]={0x97};
    write_i2c(I2C1, I2C_ACC_ADDR, ACC_CTRL_REG1_A, 1, data);
    // high-resolution output mode
    data[0]=0x08;
    write_i2c(I2C1, I2C_ACC_ADDR, ACC_CTRL_REG4_A, 1, data);
}

static void enable_magnetometer(void)
{ 
    uint8_t data[1]={0x9C};
    // enable temperature sensor, 220Hz output rate
    write_i2c(I2C1, I2C_MAG_ADDR, MAG_CRA_REG_M, 1, data);
    data[0] = MAG_GAIN_1_3;
    write_i2c(I2C1, I2C_MAG_ADDR, MAG_CRB_REG_M, 1, data);
    data[0] = 0;
    write_i2c(I2C1, I2C_MAG_ADDR, MAG_MR_REG_M, 1, data);
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;
    (void)usbd_dev;

    char buf[64];
    uint8_t data[16];

    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    if (len) {
        for (int i = 0 ; i < len ; i++) {
            if (buf[i] == 'm') {
                while (1) {
                    read_i2c(I2C1, I2C_MAG_ADDR, MAG_SR_REG_M, 1, data);
                    if (data[0] & 0x01) break;
                }
                read_i2c(I2C1, I2C_MAG_ADDR, MAG_OUT_X_H_M, 1, data);
                read_i2c(I2C1, I2C_MAG_ADDR, MAG_OUT_X_L_M, 1, data+1);
                read_i2c(I2C1, I2C_MAG_ADDR, MAG_OUT_Z_H_M, 1, data+2);
                read_i2c(I2C1, I2C_MAG_ADDR, MAG_OUT_Z_L_M, 1, data+3);
                read_i2c(I2C1, I2C_MAG_ADDR, MAG_OUT_Y_H_M, 1, data+4);
                read_i2c(I2C1, I2C_MAG_ADDR, MAG_OUT_Y_L_M, 1, data+5);
                usbd_ep_write_packet(usbd_dev, 0x82, data, 6);
                // for some reason we need to do this every time ...
                enable_magnetometer();
            } else if (buf[i] == 'a') {
                //read_i2c(I2C1, I2C_ACC_ADDR, ACC_STATUS_REG_A, 1, data);
                //my_usb_print_int(usbd_dev, data[0]);
                read_i2c(I2C1, I2C_ACC_ADDR, ACC_OUT_X_L_A, 1, data);
                read_i2c(I2C1, I2C_ACC_ADDR, ACC_OUT_X_H_A, 1, data+1);
                read_i2c(I2C1, I2C_ACC_ADDR, ACC_OUT_Y_L_A, 1, data+2);
                read_i2c(I2C1, I2C_ACC_ADDR, ACC_OUT_Y_H_A, 1, data+3);
                read_i2c(I2C1, I2C_ACC_ADDR, ACC_OUT_Z_L_A, 1, data+4);
                read_i2c(I2C1, I2C_ACC_ADDR, ACC_OUT_Z_H_A, 1, data+5);
                usbd_ep_write_packet(usbd_dev, 0x82, data, 6);
            } else if (buf[i] == 't') {
                read_i2c(I2C1, I2C_MAG_ADDR, TEMP_OUT_H_M, 1, data);
                read_i2c(I2C1, I2C_MAG_ADDR, TEMP_OUT_L_M, 1, data+1);
                usbd_ep_write_packet(usbd_dev, 0x82, data, 2);
                enable_magnetometer();
            }
        }
    }
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;
    (void)usbd_dev;

    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
                usbd_dev,
                USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                cdcacm_control_request);
}


static void usb_setup(void)
{
    /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
    rcc_usb_prescale_1();
    rcc_periph_clock_enable(RCC_USB);
    rcc_periph_clock_enable(RCC_GPIOA);

    /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF14, GPIO11| GPIO12);
}

static void i2c_setup(void)
{
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_set_i2c_clock_hsi(I2C1);

    i2c_reset(I2C1);
    /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
    gpio_set_af(GPIOB, GPIO_AF4, GPIO6| GPIO7);
    i2c_peripheral_disable(I2C1);
    //configure ANFOFF DNF[3:0] in CR1
    i2c_enable_analog_filter(I2C1);
    i2c_set_digital_filter(I2C1, I2C_CR1_DNF_DISABLED);
    //Configure PRESC[3:0] SDADEL[3:0] SCLDEL[3:0] SCLH[7:0] SCLL[7:0]
    // in TIMINGR
    i2c_100khz_i2cclk8mhz(I2C1);
    //configure No-Stretch CR1 (only relevant in slave mode)
    i2c_enable_stretching(I2C1);
    //addressing mode
    i2c_set_7bit_addr_mode(I2C1);
    i2c_peripheral_enable(I2C1);
}

int main(void)
{
    int i;

    usbd_device *usbd_dev;

    rcc_clock_setup_hsi(&rcc_hsi_8mhz[RCC_CLOCK_48MHZ]);
    i2c_setup();

    enable_accelerometer();
    enable_magnetometer();

    usb_setup();

    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings,
            3, usbd_control_buffer, sizeof(usbd_control_buffer));
    usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

    for (i = 0; i < 0x800000; i++) {
        __asm__("nop");
    }

    while (1) {
        usbd_poll(usbd_dev);
    }
}
