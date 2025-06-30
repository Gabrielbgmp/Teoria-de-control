# Teoría de Control - Péndulo Invertido

Este repositorio contiene el código y la documentación relacionados con el proyecto de un **péndulo invertido** controlado mediante un microcontrolador ESP32. Incluye una biblioteca personalizada `PIDController` que implementa tres tipos de controladores PID (básico, IIR y con filtro pasa baja) para estabilizar y controlar el sistema. El proyecto fue desarrollado como parte de la Práctica 4 del curso de Teoría de Control.

## Descripción del Proyecto

El péndulo invertido es un sistema clásico en control automático, donde un péndulo montado sobre un carro móvil debe mantenerse en posición vertical (invertida) mediante un motor DC. Este repositorio incluye:
- **Biblioteca `PIDController`**: Implementación en C++ de tres controladores PID adaptados para ESP32.
- **Código de Ejemplo**: Un sketch que demuestra el control del péndulo en dos modos: balanceo puro y balanceo con control de posición.
- **Documentación**: Instrucciones para configurar y probar el sistema.

## Características
- Controladores PID: Básico, IIR y con filtro pasa baja.
- Modos de operación: Balanceo solo y balanceo con posición (estructura en cascada).
- Interfaz serial para ajustar parámetros en tiempo real (ganancias PID, setpoint de posición).
- Frecuencia de muestreo: 100 Hz.
- Hardware: ESP32, encoder, motor DC y simulación de IMU.

## Requisitos
- **Hardware**:
  - Microcontrolador ESP32.
  - Motor DC con control PWM.
  - Encoder conectado a pines digitales (ej. pines 2 y 3).
  - IMU (simulado en el código de ejemplo; reemplazar con hardware real si aplica).
- **Software**:
  - Entorno de desarrollo Arduino (compatible con ESP32).
  - Biblioteca `PIDController` incluida en este repositorio.

## Instalación
1. Clona este repositorio:
   ```bash
   git clone https://github.com/Gabrielbgmp/Teoria-de-control.git
