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

struct pacote {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	uint8_t u;
	uint8_t sync; 
	char frase[7];
	uint8_t cabecalho; 
	uint8_t pos;
	size_t value;
};


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


void leitura_rx(void){
	struct pacote *pacote_leitura;
	pacote_leitura = k_fifo_get(&recebe, K_FOREVER);
	
}