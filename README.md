# Teoría de Control - Péndulo Invertido

Este repositorio contiene el código, la biblioteca personalizada y los recursos relacionados con el proyecto de un **péndulo invertido** controlado mediante un microcontrolador ESP32. Desarrollado como parte de la Práctica 4 del curso de Teoría de Control, el proyecto incluye la implementación de controladores PID y un modelo 3D del péndulo.

## Descripción del Proyecto

El péndulo invertido es un sistema clásico de control automático donde un péndulo montado sobre un carro móvil debe mantenerse en posición vertical usando un motor DC. Este repositorio alberga:

- **Biblioteca `PIDController`**: Implementación en C++ de tres controladores PID (básico, IIR y con filtro pasa baja) adaptada para ESP32.
- **Ejemplo de Control**: Código para controlar el péndulo en modos de balanceo puro y balanceo con posición.
- **Modelo 3D**: Archivo STL para replicar el diseño físico del péndulo.
- **Documentación**: Instrucciones y detalles del proyecto.

## Contenido del Repositorio

- **`PIDController/`**: Carpeta con los archivos `PIDController.h` y `PID.cpp` de la biblioteca.
- **`examples/PendulumControl/`**: Ejemplo de uso para el ESP32 (archivo `.ino`).
- **`pendulum.stl`**: Modelo 3D del péndulo invertido.
- **`README.md`**: Este archivo con información general.

## Características

- **Controladores PID**: 
  - Básico: Implementación estándar en tiempo discreto.
  - IIR: Con respuesta infinita por impulso.
  - Con filtro pasa baja: Para reducir ruido en el término derivativo.
- **Modos de Operación**: Balanceo del ángulo y control combinado de ángulo y posición.
- **Interfaz Serial**: Ajuste en tiempo real de ganancias PID y setpoints.
- **Frecuencia**: 100 Hz.

## Requisitos

### Hardware
- ESP32.
- Motor DC con control PWM.
- Encoder (conectado a pines digitales).
- IMU (simulada; reemplazar con hardware real si aplica).

### Software
- Arduino IDE (configurado para ESP32).
- Biblioteca `PIDController` (incluida).



