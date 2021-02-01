/******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*! \file pltf_gpio.c
 *
 *  \author Shikha Singh
 *
 *  \brief Implementation of GPIO specific functions and its helper functions.
 *
 */

/*
 ******************************************************************************
 * INCLUDES
 ******************************************************************************
 */
#include <stdio.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <errno.h>
#include "platform.h"

#include <linux/gpio.h>
#include <sys/ioctl.h>
/*
 ******************************************************************************
 * DEFINES
 ******************************************************************************
 */
/* Max size of file path to access */
#define SIZE    60

/*
 ******************************************************************************
 * STATIC VARIABLES
 ******************************************************************************
 */
static int isGPIOInit    = 0;
static int fd_readGPIO    = 0;
// static pthread_mutex_t lock;
// static pthread_mutex_t lockWorker;

pthread_mutex_t lock;
pthread_mutex_t lockWorker;

/*
 ******************************************************************************
 * GLOBAL AND HELPER FUNCTIONS
 ******************************************************************************
 */
ReturnCode gpio_init(void)
{
  /*svs: This is first function called from main() */
    struct gpioevent_request req;
    int ret = 0;
    char gpiochip_file[20];

    sprintf(gpiochip_file,"/dev/gpiochip%d",ST25R_INT_PORT);
    // sprintf(gpiochip_file,"/dev/gpiochip%d",0);
    int fd = open(gpiochip_file, 0);
    if (fd < 0) {
        ret = -errno;
        fprintf(stderr, "Failed to open %s\n", gpiochip_file);
        return ret;
    }

    memset(&req, 0, sizeof(req));
    req.lineoffset = ST25R_INT_PIN;
    // req.lineoffset = 14;
    req.handleflags = GPIOHANDLE_REQUEST_INPUT;
    req.eventflags = GPIOEVENT_REQUEST_RISING_EDGE;
    strcpy(req.consumer_label,"spirit1_irq");
    ret = ioctl(fd,  GPIO_GET_LINEEVENT_IOCTL, &req);
    if(ret < 0)
    {
        ret = -errno;
        fprintf(stderr, "Failed to ioctl gpio%d\n",ST25R_INT_PIN);
        close(fd);
        return ret;
    } /*svs commented to test*/

    ret = pthread_mutex_init(&lock, NULL);
    if (ret != 0)
    {
        printf("Error: mutex init to protect interrupt status is failed\n");
        close(req.fd);
        close(fd);
        return ret;
    }

    ret = pthread_mutex_init(&lockWorker, NULL);
    if (ret != 0)
    {
        printf("Error: mutex init to protect worker is failed\n");
        close(req.fd);
        close(fd);
        return ret;
    }
    close(fd);
    fd_readGPIO = req.fd;
    isGPIOInit = 1;
    return ret;

}


GPIO_PinState gpio_readpin(int port, int pin_no)
{
    int ret;
    struct gpiohandle_data input_values;
    int gpio_fd;
    /* IRQ pin is already opened */
    bool st25r3911_irq_pin = (port == ST25R_INT_PORT) && (pin_no == ST25R_INT_PIN);

    /* First check if GPIOInit is done or not */
    if (!isGPIOInit) {
        fprintf(stderr,"GPIO is not initialized\n");
        return ERR_WRONG_STATE;
    }

    if(st25r3911_irq_pin)
    {
        /* use irq gpio fd */
        gpio_fd = fd_readGPIO;
    } else {
        /* open the gpio for reading */
        char gpiochip_file[20];
        struct gpiohandle_request req;
        int fd, ret;

        sprintf(gpiochip_file,"/dev/gpiochip%d",port);
        fd = open(gpiochip_file, 0);
        if (fd < 0) {
            ret = -errno;
            fprintf(stderr, "Failed to open %s\n", gpiochip_file);
            return ERR_IO;
        }

        memset(&req, 0, sizeof(req));
        req.flags = GPIOHANDLE_REQUEST_INPUT;
        req.lines = 1;
        req.lineoffsets[0] = pin_no;
        strcpy(req.consumer_label, "st25r391x_in");
        ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
        if (ret < 0) {
            ret = -errno;
            fprintf(stderr, "Failed to open %s pin%d ret %d\n", gpiochip_file,pin_no, ret);
            close(fd);
            return ERR_IO;
        }
        close(fd);
        gpio_fd = req.fd;
    }

    ret = ioctl(fd_readGPIO, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &input_values);
    /* first close fd if required */
    if(!st25r3911_irq_pin)
    {
        close(gpio_fd);
    }
    /* then check the ioctl status */
    if (ret < 0) {
        fprintf(stderr,"Error: while reading GPIO pin state\n");
        return ERR_IO;
    }

    if (input_values.values[0] == 0) {
        return GPIO_PIN_RESET;
    } else {
        return GPIO_PIN_SET;
    }
}

void platformIsr()
{
    // st25r3911CheckForReceivedInterrupts();
    printf("\nSVS LOG : Interrupt occured\n");

    P2PInterruptHandler();

    // if (NULL != st25r3911interrupt.callback)
    // {
    //     st25r3911interrupt.callback();
    // }
}

void* pthread_func()
{
    int ret = 0;
    struct gpioevent_data event;
    struct pollfd poll_fd;
    poll_fd.fd = fd_readGPIO;
    poll_fd.events = POLLIN | POLLPRI;

    /* First check if GPIOInit is done or not */
    if (!isGPIOInit) {
        fprintf(stderr,"GPIO is not initialized\n");
        return NULL;
    }

    /* poll interrupt live forever */
    for(;;)
    {
      printf("\nSVS LOG: Thread pthread_func() Running \n");
      ssize_t s = read(fd_readGPIO, &event, sizeof(event));
      printf("\nSVS : Thread pthread_func() INTERRUPT received \n");
        /* Call RFAL Isr */
        platformIsr();
    }
}

ReturnCode interrupt_init(void)
{

    pthread_t intr_thread;
    struct sched_param params;
    int ret;

    // /* create a pthread to poll for interrupt */
    // ret = pthread_create(&intr_thread, NULL, pthread_func, NULL);
    // if (ret) {
    //     fprintf(stderr, "Error: poll thread creation %d\n", ret);
    //     return ERR_IO;
    // }

    // /* Assign highest priority to polling thread */
    // params.sched_priority = sched_get_priority_max(SCHED_FIFO);
    // ret = pthread_setschedparam(intr_thread, SCHED_FIFO, &params);
    // if (ret) {
    //     fprintf(stderr,"Error: assigning high priority to polling thread\n");
    //     return ERR_IO;
    // }

    return ERR_NONE;
}

void gpio_set_value(int port, int pin_no, uint8_t value)
{
    char gpiochip_file[20];
    struct gpiohandle_request req;
    struct gpiohandle_data data;
    int fd, ret;
    int trials = 3;
    bool retry = 0;

    sprintf(gpiochip_file,"/dev/gpiochip%d",port);
    fd = open(gpiochip_file, 0);
    if (fd < 0) {
        ret = -errno;
        fprintf(stderr, "Failed to open %s\n", gpiochip_file);
        return;
    }

    /*svs: write  commnets within all functions */

    memset(&req, 0, sizeof(req));
    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.lines = 1;
    req.lineoffsets[0] = pin_no;
    req.default_values[0] = (value != 0);
    strcpy(req.consumer_label, "st25r391x_out");
    ret = ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req);
    if (ret < 0) {
        ret = -errno;
        /* the gpio is busy */
        /* fprintf(stderr, "Failed to open %s pin%d ret %d\n", gpiochip_file,pin_no, ret); */
        close(fd);
        return;
    }
    close(fd);

    memset(&data, 0, sizeof(data));
    data.values[0] = (value != 0);
    ret = ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data);
    if (ret < 0) {
        ret = -errno;
        fprintf(stderr, "Failed to set %s pin%d\n", gpiochip_file,pin_no);
    }
    close(req.fd);
}


void gpio_set(int port, int pin_no)
{
    gpio_set_value(port,pin_no,1);
}

void gpio_clear(int port, int pin_no)
{
    gpio_set_value(port,pin_no,0);
}

void pltf_protect_interrupt_status(void)
{
    //pthread_mutex_lock(&lock);
}

void pltf_unprotect_interrupt_status(void)
{
    //pthread_mutex_unlock(&lock);
}

void pltf_protect_worker(void)
{
    //pthread_mutex_lock(&lockWorker);
}

void pltf_unprotect_worker(void)
{
    //pthread_mutex_unlock(&lockWorker);
}
