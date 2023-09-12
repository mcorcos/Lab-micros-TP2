#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "source/protocols/uart.h" // Asegúrate de incluir el archivo de encabezado de tu código UART aquí

#define UART_TEST_ID 0 // El ID de UART que deseas probar (ajústalo según tu configuración)

int main() {
    // Inicializa el UART con la configuración predeterminada o la que desees probar
    uart_cfg_t config = {
        .baudrate = UART_HAL_DEFAULT_BAUDRATE, // Puedes ajustar la velocidad aquí
    };
    uartInit(UART_TEST_ID, config);

    // Crea un mensaje de prueba
    char testMessage[] = "Hello, UART!\r\n";

    // Envia el mensaje a través del UART
    uartWriteMsg(UART_TEST_ID, testMessage, sizeof(testMessage) - 1); // Resta 1 para excluir el carácter nulo '\0'

    // Espera a recibir una respuesta a través del UART
    while (!uartIsRxMsg(UART_TEST_ID)) {
        // Espera a que se complete la recepción
    }

    // Lee la respuesta del UART
    uint8_t rxBuffer[128]; // Ajusta el tamaño del búfer según tus necesidades
    uint8_t bytesRead = uartReadMsg(UART_TEST_ID, rxBuffer, sizeof(rxBuffer));
    rxBuffer[bytesRead] = '\0'; // Asegúrate de terminar la cadena con '\0' para imprimir

    // Imprime la respuesta recibida
    printf("Received: %s\n", rxBuffer);

    return 0;
}
