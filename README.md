\chapter{Práctica 4: Implementación de Controladores PID en ESP32 para Péndulo Invertido}

\section{Enunciado}
Esta práctica consistió en implementar tres pseudocódigos de controladores PID (básico, IIR y con filtro pasa baja) en un microcontrolador ESP32 para el proyecto del péndulo invertido, conforme a lo descrito en el capítulo 6. Se requirió integrar estos controladores en una biblioteca personalizada de control PID, siguiendo las guías disponibles en \url{https://www.arduino.cc/en/Hacking/libraryTutorial} y \url{https://sp.arduino-france.site/crear-libreria/}, adaptadas al entorno de desarrollo de Arduino para ESP32. Además, se realizaron pruebas de funcionamiento con el péndulo invertido construido y se publicó el código de las librerías en el repositorio de GitHub del proyecto.

\section{Resolución}

\subsection{Implementación de los Controladores PID}
Se desarrollaron tres tipos de controladores PID para su implementación en el microcontrolador ESP32:

- **PID Básico**: Este controlador se implementó como una versión estándar en tiempo discreto, utilizando las ecuaciones fundamentales de control proporcional, integral y derivativo.
- **PID IIR**: Se diseñó un controlador basado en un filtro de respuesta infinita por impulso (IIR), optimizado para mejorar la respuesta dinámica del sistema.
- **PID con Filtro Pasa Baja**: Esta versión incorporó un filtro pasa baja en el término derivativo para mitigar el efecto del ruido de alta frecuencia en las mediciones.

Estos controladores se programaron en el entorno de Arduino, adaptados específicamente para el ESP32, siguiendo los pseudocódigos proporcionados en el enunciado de la práctica.

\subsection{Creación de la Biblioteca PID}
Se desarrolló una biblioteca personalizada en C++ llamada `PIDController`, siguiendo las directrices del tutorial de Arduino para la creación de bibliotecas. Esta biblioteca fue adaptada para el ESP32 e incluyó las siguientes características:

- Una enumeración para seleccionar entre los tres tipos de controladores (básico, IIR y con filtro pasa baja).
- Métodos para inicializar los controladores, calcular las salidas de control y ajustar parámetros como ganancias (proporcional, integral y derivativa) y tiempo de muestreo.
- Funcionalidades adicionales para reiniciar el estado del controlador y consultar sus variables internas.

La biblioteca se estructuró en dos archivos principales: un archivo de cabecera y un archivo de implementación, ambos disponibles en el repositorio del proyecto.

\subsection{Pruebas con el Péndulo Invertido}
Las pruebas se llevaron a cabo con el péndulo invertido construido, utilizando un programa principal para controlar el sistema. Las pruebas abarcaron:

- **Estabilización del Péndulo**: Se logró mantener el péndulo en posición vertical mediante el control del ángulo, utilizando los controladores PID implementados.
- **Control en Cascada**: Se empleó una estructura en cascada para controlar simultáneamente el ángulo del péndulo y la posición del carro, ajustando los parámetros en tiempo real a través de comandos seriales.

Los resultados indicaron que el controlador con filtro pasa baja ofreció una mayor estabilidad en presencia de ruido, mientras que el PID básico fue efectivo en condiciones ideales. Las pruebas se realizaron ajustando dinámicamente las ganancias para optimizar el rendimiento del sistema.

\subsection{Publicación en GitHub}
El código desarrollado, incluyendo la biblioteca `PIDController`, los ejemplos de uso y el modelo 3D del péndulo, se publicó en el repositorio de GitHub del proyecto, accesible en \url{https://github.com/Gabrielbgmp/Teoria-de-control}. El repositorio contiene:

- Archivos de la biblioteca personalizada.
- Un ejemplo de uso para el péndulo invertido.
- Un archivo STL con el modelo 3D del péndulo, útil para replicar el hardware.
- Un archivo `README.md` con instrucciones detalladas y una descripción del proyecto.

La publicación se completó exitosamente, con un total de cuatro commits registrados hasta el 30 de junio de 2025, asegurando que todo el material esté disponible para su consulta y uso posterior.
