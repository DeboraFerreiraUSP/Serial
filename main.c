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

K_FIFO_DEFINE (envia);
K_FIFO_DEFINE (recebe);
K_MUTEX_DEFINE (mutex_tx);
K_CONDVAR_DEFINE (condvar_tx);
struct pacote {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	uint8_t u;
	uint8_t sync; 
	char frase[7];
	uint8_t cabecalho; 
	uint8_t pos;
	size_t value;
};

const struct device *sb = DEVICE_DT_GET(DT_NODELABEL(gpiob));


void teclado (void){
	struct pacote pacote_teclado;
	pacote_teclado.u = 0b0101010101;
	pacote_teclado.sync = 0b00010110;
	pacote_teclado.pos = 0b00010111;

	while(1){
		printfk("Digite uma frase: ");
		fgets(pacote_teclado.frase, sizeof(pacote_teclado.frase), stdin);
		size_t comprimento = strlen(pacote_teclado.frase); 
		pacote_teclado.cabecalho = 0b01101000;


		if (pacote_teclado.frase[comprimento - 1] == '\n') { 
			pacote_teclado.frase[comprimento - 1] = '\0'; 
			comprimento--; 
		}
		if (comprimento<=7) {
			pacote_teclado.cabecalho = pacote_teclado.cabecalho | comprimento;		
		}
		
		k_fifo_put(&envia, &pacote_teclado);
	}
}


void tx(void){
	struct pacote *tx;
	int aux=0;
	gpio_pin_set(sb, 0x3, aux);
	if (tx = k_fifo_get(&envia, K_FOREVER)!=NULL){
		for (int i=1; i<= 8; i++){
			aux = (*tx).u && 0b1;
			(*tx).u = (*tx).u >> 1;
			k_msleep(10);
			gpio_pin_set(sb, 0x3, aux);
		}for (int i=1; i<= 8; i++){
			aux = (*tx).sync && 0b1;
			(*tx).sync = (*tx).sync >> 1;
			k_msleep(10);
			gpio_pin_set(sb, 0x3, aux);
		}for (int i=1; i<= 8; i++){
			aux = (*tx).cabecalho && 0b1;
			(*tx).cabecalho = (*tx).cabecalho >> 1;
			k_msleep(10);
			gpio_pin_set(sb, 0x3, aux);
		}for (int i=0; i<((*tx).cabecalho && 0b111); i++) {
			for (int j=0; j<8;j++) {
				aux = (*tx).frase[i] && 0b1;
				(*tx).frase[i] = (*tx).frase[i] >> 1;
				k_msleep(10);
				gpio_pin_set(sb, 0x3, aux);
		}
		}for (int i=1; i<= 8; i++){
			aux = (*tx).cabecalho && 0b1;
			(*tx).cabecalho = (*tx).cabecalho >> 1;
			k_msleep(10);
			gpio_pin_set(sb, 0x3, aux);
		}for (int i=1; i<= 8; i++){
			aux = (*tx).pos && 0b1;
			(*tx).pos = (*tx).pos >> 1;
			k_msleep(10);
			gpio_pin_set(sb, 0x3, aux);
		}
			aux = 0;
			gpio_pin_set(sb, 0x3, aux);
		}
}



void rx(void) {
	int32_t buffer;
	struct pacote rx;
	
	while (buffer != 0b0101010101) {
		k_msleep();
	}
}
void main(void) {
	gpio_pin_configure(sb, 0x3, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(sb, 0x2, GPIO_INPUT);
}