/*
 * Copyright (c) 2017 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/__assert.h>
#include <string.h>
#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_shell_uart)// define a UART




//K_MSGQ_DEFINE(uart_msgq, 32, 10, 4);// define a mensage queue (=fifo)

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


static char echo_buf[7];// declara o buffer que guarda os bits recebidos
static int echo_buf_pos;// declara a variável para posição do buffer

K_MSGQ_DEFINE (teclado_msgq, 128, 10, 4);// define a FIFO que envia dados

uint8_t buffer;// declara um buffer de 8 bits
int i=0;// contador para a posição dos bits a serem transmitidos
int f=0;
int j=0;// contador para a posição dentro do vetor frase
int d=0;
int k=0, m=0, count_buf4 = 0, recebendo = 0, ordem = 0;
int tamanhobits_tx, tamanhobits_rx;// declara uma variável inteira para armazenar o número total de bits do pacote

    uint8_t u = 0b01010101;// declara U
    uint8_t sync = 0b00010110;// declara o sync
    char frase[7] = {0};// declara um vetor para armazenar a mensagem
    uint8_t cabecalho = 0b11010000; // declara o cabeçalho -> ID (11010) + tamanho da mensagem
    uint8_t pos = 0b00010111;// declara pos, que indica o fim da transmissão
    
    uint8_t u_rx = 0b01010101;// declara o U a ser recebido pelo RX
    uint8_t sync_rx = 0b00010110;// declara o sync a ser recebido pelo RX
    char frase_rx[7] = {0};// declara um vetor para armazenar a mensagem a ser recebida pelo RX 
    uint8_t cabecalho_rx = 0b11010000; 
    uint8_t pos_rx = 0b00010111;
    uint8_t buffer_bits4; 

const struct device *sb = DEVICE_DT_GET(DT_NODELABEL(gpiob));// 







void tx(void){// função que realiza a transmissão 
    int aux=0;// declara auxiliar para 
    if (i==0){
        gpio_pin_set(sb, 0x3, 0);// coloca a gpio em 0
        if (k_msgq_get(&teclado_msgq, &frase, K_NO_WAIT) == 0) {// coloca as informações contidas em teclado_msgq no vetor frase
            cabecalho = cabecalho | strlen(frase);// altera o valor dos três últimos bits do cabeçalho para definir o tamanho em bytes da mensagem
            tamanhobits_tx = 8*(cabecalho & 0b111);// calcula o tamanho em bits da mensagem
            i += 1;// soma 1 no contador
        }
    } else if (i == 1) {
        gpio_pin_set(sb, 0X3, 1);// coloca gpio em 1 (start bit)
        i += 1;// soma 1 no contador
    } 
    else if (i < 10) {// entra nessa condição até seja maior ou igual a 10
        aux = u & 0b1;// coloca em aux o valor do bit menos significativo de u
        u = u >> 1;// desloca "u" um bit para a direita para na próxima iteração pegar o próximo bit
        gpio_pin_set(sb, 0X3, aux);// coloca na gpio o mesmo valor obtido em aux
        
        i+=1;//soma 1 no somador 
    } else if (i < 18) {// entra nessa condição até seja maior ou igual a 18
        aux = sync & 0b1;// coloca em aux o valor do bit menos significativo de sync
        sync = sync >> 1;// desloca "sync" um bit para a direita para na próxima iteração pegar o próximo bit
        gpio_pin_set(sb, 0X3, aux);// coloca na gpio o mesmo valor obtido em aux
        i+=1;// soma 1 no somador
    } else if (i < 26) {// entra nessa condição até seja maior ou igual a 26
        aux = cabecalho & 0b1;// coloca em aux o valor do bit menos significativo do cabeçalho
        cabecalho = cabecalho >> 1;// desloca "cabeçalho" um bit para a direita para na próxima iteração pegar o próximo bit
        gpio_pin_set(sb, 0X3, aux);// coloca na gpio o mesmo valor obtido em aux
        i+=1;// soma 1 no somador
        
    } else if (i < (26 + tamanhobits_tx)) {// entra nessa condição até seja maior ou igual a 26+ tamanho da mensagem
        if (j!=(tamanhobits_tx/8)) {// entra nessa condição se j é diferente do tamanho da mensagem
            aux = frase[j] & 0b1;// coloca em aux o valor do bit menos significativo do byte da posição j
            frase[j] = frase[j] >> 1; 
            gpio_pin_set(sb, 0X3, aux);
            //printk(" %d ",gpio_pin_get(sb,0x1));
            //printk("%d ", aux);

            i+=1;
            k+=1;
        }
        if (k == 8) {
            //printk("|");
            k = 0;
            j++;
        }
    } else if (i < (34 + tamanhobits_tx)) {
        aux = pos & 0b1;
        pos = pos >> 1;
        gpio_pin_set(sb, 0X3, aux);
        i+=1;
    } else {
        i=0;
        //gpio_pin_set(sb, 0X3, 0);
    }
    
}


K_TIMER_DEFINE(tx_call, tx, NULL);



void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;
   
    if (!uart_irq_update(uart_dev)) {
        return;
    }

    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    /* read until FIFO empty */
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if ((c == '\n' || c == '\r') && echo_buf_pos > 0) {
            /* terminate string */
            echo_buf[echo_buf_pos] = '\0';
    
            /* if queue is full, message is silently dropped */
            k_msgq_put(&teclado_msgq, &echo_buf, K_NO_WAIT);
            /* reset the buffer (it was copied to the msgq) */
            echo_buf_pos = 0;
        } else if (echo_buf_pos < (sizeof(echo_buf) - 1)) {
            echo_buf[echo_buf_pos++] = c;
            
        }
        /* else: characters beyond buffer size are dropped */
    }
}

void mensagem() {
    printk("Você recebeu uma mensagem:\n");
    for (int s = 0; s < strlen(frase_rx); s++) {
        printk("%c", frase_rx[s]);
    } 
}

void rx(void) {
    if((f < 8) && (ordem == 0)) {
        f++;
    } else if(ordem == 0) {
        if ((buffer ^ u_rx )==0) {
            ordem++;
            f++;
            printk("| achou o U |");
        } else {
            recebendo = 0;
            ordem = 0;
            f = 0;
            printk("entrei");
        }
    } else if (f < 16) {
        f++;
    } else if(ordem == 1) {
        if ((buffer ^sync_rx)==0) {
            ordem++;
            f++;
            printk("| achou o SYNC |");
        } else {
            recebendo = 0;
            ordem = 0;
            f = 0;
        }
    } else if (f < 24) {
        f++;
    } else if(ordem == 2) {
        if (((buffer & 0b11111000) ^ (cabecalho_rx)) == 0) {
            ordem++;
            f++;
            tamanhobits_rx = (buffer & 0b00000111)*8;
            printk("| achou o cabecalho: %d |", tamanhobits_rx);
        } else {
            recebendo = 0;
            ordem = 0;
            f = 0;
        }
    } else if(ordem == 3) {
            if((f%8 == 0) && (d!= (tamanhobits_rx/8))){
                frase_rx[d] = buffer;
                printk("| achou a frase: %c |", frase_rx[d]);
                d++;
            }
            if(d == (tamanhobits_rx/8)){
                ordem++;
                d = 0;
            }
            f++; 
    } else if (f < (32 + tamanhobits_rx)) {
        f++;
    } else if(ordem == 4) {
        if ((buffer ^ pos_rx) == 0) {
            recebendo = 0;
            ordem = 0;
            printk("| achou o pos |");
            mensagem();
        } else {
            recebendo = 0;
            ordem = 0;
            f = 0;
        }
    } else {
        recebendo = 0;
        ordem = 0;
        f = 0;
    }
}




void rx_buffer(void) {
    buffer_bits4 = (buffer_bits4 >> 1) | (gpio_pin_get(sb,0x1) << 7);
    if (recebendo > 0) {
        if (((buffer_bits4 & 0b01100000) == 0b01100000) && (count_buf4%4 == 0)) {
            buffer = (buffer >> 1) | 0b10000000;
            count_buf4 = 0;
            rx();
        } else if (((buffer_bits4 & 0b01100000) == 0b00000000) && (count_buf4%4 == 0)) {
            buffer = (buffer >> 1); 
            count_buf4 = 0;
            rx();
        }
        count_buf4++;
    }
    if ((((buffer_bits4 & 0b11100000) == 0b11100000) || ((buffer_bits4 & 0b01110000) == 0b01110000) || ((buffer_bits4 & 0b11110000) == 0b11110000)) && (recebendo == 0)) {
        recebendo++;
    }
}


K_TIMER_DEFINE(rx_buffer_call, rx_buffer, NULL);            

void main(void) {
    k_timer_start(&rx_buffer_call, K_NO_WAIT, K_MSEC(10));
    k_timer_start(&tx_call, K_NO_WAIT, K_MSEC(40));
    gpio_pin_configure(sb, 0x3, GPIO_OUTPUT_LOW);
    gpio_pin_configure(sb, 0x1, GPIO_INPUT | GPIO_ACTIVE_HIGH);
    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);
    printk("Digite uma mensagem\n:");    
}
