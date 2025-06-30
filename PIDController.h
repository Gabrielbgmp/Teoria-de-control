#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/**
 * @brief Enumeración para los tipos de controlador PID disponibles
 */
enum PIDType {
    PID_BASIC,      // PID básico en tiempo discreto
    PID_IIR,        // PID implementado como filtro IIR
    PID_FILTERED    // PID con filtro pasa baja en el término derivativo
};

/**
 * @brief Clase para implementar controladores PID en tiempo discreto
 * 
 * Esta biblioteca implementa tres tipos de controladores PID:
 * 1. PID básico en tiempo discreto
 * 2. PID como filtro IIR
 * 3. PID con filtro pasa baja en el término derivativo
 */
class PIDController {
private:
    // Parámetros del controlador
    double Kp, Ki, Kd;          // Ganancias del controlador
    double dt;                  // Tiempo de muestreo
    PIDType pidType;            // Tipo de controlador
    
    // Variables para PID básico
    double previous_error;
    double integral;
    
    // Variables para PID IIR
    double A0, A1, A2;          // Coeficientes del filtro IIR
    double error[3];            // Buffer de errores [0]=actual, [1]=anterior, [2]=anterior-1
    
    // Variables para PID con filtro
    double A0d, A1d, A2d;       // Coeficientes derivativos
    int N;                      // Factor del filtro (3 <= N <= 10)
    double tau;                 // Constante de tiempo del filtro
    double alpha, alpha_1, alpha_2; // Coeficientes del filtro
    double d0, d1;              // Variables auxiliares derivativo
    double fd0, fd1;            // Variables auxiliares filtro
    
    // Variables comunes
    double output;              // Salida del controlador
    double u0;                  // Valor inicial del actuador
    bool initialized;           // Estado de inicialización
    
public:
    /**
     * @brief Constructor de la clase PIDController
     * @param kp Ganancia proporcional
     * @param ki Ganancia integral
     * @param kd Ganancia derivativa
     * @param sample_time Tiempo de muestreo en segundos
     * @param type Tipo de controlador PID (por defecto PID_BASIC)
     * @param filter_factor Factor N para el filtro pasa baja (solo para PID_FILTERED)
     */
    PIDController(double kp, double ki, double kd, double sample_time, 
                  PIDType type = PID_BASIC, int filter_factor = 5);
    
    /**
     * @brief Inicializa el controlador PID
     * @param initial_output Valor inicial del actuador
     */
    void begin(double initial_output = 0.0);
    
    /**
     * @brief Calcula la salida del controlador PID
     * @param setpoint Valor de referencia
     * @param measured_value Valor medido del proceso
     * @return Salida del controlador
     */
    double compute(double setpoint, double measured_value);
    
    /**
     * @brief Actualiza los parámetros del controlador
     * @param kp Nueva ganancia proporcional
     * @param ki Nueva ganancia integral
     * @param kd Nueva ganancia derivativa
     */
    void setTunings(double kp, double ki, double kd);
    
    /**
     * @brief Actualiza el tiempo de muestreo
     * @param sample_time Nuevo tiempo de muestreo en segundos
     */
    void setSampleTime(double sample_time);
    
    /**
     * @brief Cambia el tipo de controlador PID
     * @param type Nuevo tipo de controlador
     * @param filter_factor Factor N para el filtro (solo para PID_FILTERED)
     */
    void setPIDType(PIDType type, int filter_factor = 5);
    
    /**
     * @brief Reinicia el controlador (limpia integrales y historiales)
     */
    void reset();
    
    /**
     * @brief Obtiene la salida actual del controlador
     * @return Salida actual
     */
    double getOutput() const;
    
    /**
     * @brief Obtiene los parámetros actuales del controlador
     * @param kp Referencia para almacenar Kp
     * @param ki Referencia para almacenar Ki
     * @param kd Referencia para almacenar Kd
     */
    void getTunings(double &kp, double &ki, double &kd) const;
    
    /**
     * @brief Obtiene el tiempo de muestreo actual
     * @return Tiempo de muestreo en segundos
     */
    double getSampleTime() const;
    
    /**
     * @brief Obtiene el tipo de controlador actual
     * @return Tipo de controlador
     */
    PIDType getPIDType() const;

private:
    /**
     * @brief Inicializa los coeficientes según el tipo de controlador
     */
    void initializeCoefficients();
    
    /**
     * @brief Implementa el PID básico en tiempo discreto
     * @param error Error actual (setpoint - measured_value)
     * @return Salida del controlador
     */
    double computeBasicPID(double error);
    
    /**
     * @brief Implementa el PID como filtro IIR
     * @param error Error actual
     * @return Salida del controlador
     */
    double computeIIRPID(double error);
    
    /**
     * @brief Implementa el PID con filtro pasa baja
     * @param error Error actual
     * @return Salida del controlador
     */
    double computeFilteredPID(double error);
};

#endif // PID_CONTROLLER_H