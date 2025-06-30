#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd, double sample_time, 
                           PIDType type, int filter_factor) 
    : Kp(kp), Ki(ki), Kd(kd), dt(sample_time), pidType(type), N(filter_factor) {
    
    // Inicializar variables
    previous_error = 0.0;
    integral = 0.0;
    output = 0.0;
    u0 = 0.0;
    initialized = false;
    
    // Inicializar arrays
    for(int i = 0; i < 3; i++) {
        error[i] = 0.0;
    }
    
    // Variables del filtro
    d0 = d1 = fd0 = fd1 = 0.0;
    
    // Validar parámetros
    if(dt <= 0.0) dt = 0.01; // Valor por defecto de 10ms
    if(N < 3) N = 3;
    if(N > 10) N = 10;
    
    initializeCoefficients();
}

void PIDController::begin(double initial_output) {
    u0 = initial_output;
    output = initial_output;
    reset();
    initialized = true;
}

double PIDController::compute(double setpoint, double measured_value) {
    if(!initialized) {
        begin();
    }
    
    double current_error = setpoint - measured_value;
    
    switch(pidType) {
        case PID_BASIC:
            output = computeBasicPID(current_error);
            break;
        case PID_IIR:
            output = computeIIRPID(current_error);
            break;
        case PID_FILTERED:
            output = computeFilteredPID(current_error);
            break;
    }
    
    return output;
}

double PIDController::computeBasicPID(double error) {
    // Implementación del pseudocódigo básico
    integral += error * dt;
    double derivative = (error - previous_error) / dt;
    
    double result = Kp * error + Ki * integral + Kd * derivative;
    
    previous_error = error;
    
    return result;
}

double PIDController::computeIIRPID(double current_error) {
    // Desplazar el historial de errores
    error[2] = error[1];
    error[1] = error[0];
    error[0] = current_error;
    
    // Calcular salida usando coeficientes IIR
    double result = A0 * error[0] + A1 * error[1] + A2 * error[2];
    
    return result;
}

double PIDController::computeFilteredPID(double current_error) {
    // Desplazar el historial de errores
    error[2] = error[1];
    error[1] = error[0];
    error[0] = current_error;
    
    // Parte PI
    output += A0 * error[0] + A1 * error[1];
    
    // Parte derivativa filtrada
    d1 = d0;
    d0 = A0d * error[0] + A1d * error[1] + A2d * error[2];
    
    fd1 = fd0;
    fd0 = alpha_1 * (d0 + d1) - alpha_2 * fd1;
    
    return output + fd0;
}

void PIDController::setTunings(double kp, double ki, double kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    initializeCoefficients();
}

void PIDController::setSampleTime(double sample_time) {
    if(sample_time > 0.0) {
        dt = sample_time;
        initializeCoefficients();
    }
}

void PIDController::setPIDType(PIDType type, int filter_factor) {
    pidType = type;
    if(type == PID_FILTERED) {
        N = filter_factor;
        if(N < 3) N = 3;
        if(N > 10) N = 10;
    }
    initializeCoefficients();
    reset();
}

void PIDController::reset() {
    // Limpiar variables del PID básico
    previous_error = 0.0;
    integral = 0.0;
    
    // Limpiar historial de errores
    for(int i = 0; i < 3; i++) {
        error[i] = 0.0;
    }
    
    // Limpiar variables del filtro
    d0 = d1 = fd0 = fd1 = 0.0;
    
    // Reinicializar salida
    output = u0;
}

void PIDController::initializeCoefficients() {
    switch(pidType) {
        case PID_BASIC:
            // No necesita coeficientes especiales
            break;
            
        case PID_IIR:
            // Coeficientes según ecuación (4.8)
            A0 = Kp + Ki * dt + Kd / dt;
            A1 = -Kp - 2.0 * Kd / dt;
            A2 = Kd / dt;
            break;
            
        case PID_FILTERED:
            // Coeficientes para la parte PI
            A0 = Kp + Ki * dt;
            A1 = -Kp;
            
            // Coeficientes para la parte derivativa
            A0d = Kd / dt;
            A1d = -2.0 * Kd / dt;
            A2d = Kd / dt;
            
            // Parámetros del filtro pasa baja
            if(Kp != 0.0) {
                tau = Kd / (Kp * N);
            } else {
                tau = 0.01; // Valor por defecto para evitar división por cero
            }
            
            alpha = dt / (2.0 * tau);
            alpha_1 = alpha / (alpha + 1.0);
            alpha_2 = (alpha - 1.0) / (alpha + 1.0);
            break;
    }
}

double PIDController::getOutput() const {
    return output;
}

void PIDController::getTunings(double &kp, double &ki, double &kd) const {
    kp = Kp;
    ki = Ki;
    kd = Kd;
}

double PIDController::getSampleTime() const {
    return dt;
}

PIDType PIDController::getPIDType() const {
    return pidType;
}