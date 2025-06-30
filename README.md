# Teoría de Control - Péndulo Invertido

Este repositorio contiene el código, la biblioteca personalizada, los recursos y el diseño de la PCB relacionados con el proyecto de un **péndulo invertido** controlado mediante un microcontrolador ESP32. Desarrollado como parte de la Prácticas 4 y 6 del curso de Teoría de Control, el proyecto incluye la implementación de controladores PID y un modelo 3D del péndulo, junto con el diseño de la placa de circuito impreso.

## Descripción del Proyecto

El péndulo invertido es un sistema clásico de control automático donde un péndulo montado sobre un carro móvil debe mantenerse en posición vertical usando un motor. Este repositorio alberga:

- **Biblioteca `PIDController`**: Implementación en C++ de tres controladores PID (básico, IIR y con filtro pasa baja) adaptada para ESP32.
- **Ejemplo de Control**: Código para controlar el péndulo en modos de balanceo puro y balanceo con posición.
- **Modelo 3D**: Archivo STL para replicar el diseño físico del péndulo.
- **Diseño PCB**: Esquema y diseño de la placa de circuito impreso que integra los componentes necesarios para el sistema.
- **Documentación**: Instrucciones y detalles del proyecto.

## Contenido del Repositorio

- **`PIDController/`**: Carpeta con los archivos `PIDController.h` y `PID.cpp` de la biblioteca.
- **`examples/PendulumControl/`**: Ejemplo de uso para el ESP32 (archivo `.ino`).
- **`pendulum.stl`**: Modelo 3D del péndulo invertido.
- **`pcb/`**: Carpeta con los archivos del diseño de la PCB .
- **`README.md`**: Este archivo con información general.

## Características

- **Controladores PID**: 
  - Básico: Implementación estándar en tiempo discreto.
  - IIR: Con respuesta infinita por impulso.
  - Con filtro pasa baja: Para reducir ruido en el término derivativo.
- **Modos de Operación**: Balanceo del ángulo y control combinado de ángulo y posición.
- **Interfaz Serial**: Ajuste en tiempo real de ganancias PID y setpoints.
- **Frecuencia**: 100 Hz.
- **PCB**: Diseño personalizado para integrar el ESP32 y demas componentes.

## Requisitos

### Hardware
- ESP32.
- Impresión 3D.
- Componentes para ensamblar la PCB diseñada.

### Software
- Arduino IDE (configurado para ESP32).
- Biblioteca `PIDController` (incluida).
- Software de diseño PCB para visualizar o modificar el diseño.
