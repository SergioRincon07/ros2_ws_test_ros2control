# Demostraciones de ros2_control

Este repositorio proporciona ejemplos de funcionalidades y capacidades del marco de trabajo `ros2_control`.
Consiste en implementaciones simples que demuestran diferentes conceptos.


## Contenido

Los siguientes ejemplos son parte de este repositorio de demostración:

* Ejemplo 1: [*RRBot*](test1_ros2control)

   *RRBot* es un robot el cual tien una arricuculacion la cual podemos enviar le la posicion en la que se desee.


* Ejemplo 2: [*RRBot 2.0*](test2_ros2control)

   *RRBot 2.0* es muy parecido al anterior pero ahora en vez de una articulacion se tiene una llanta el cual su movimiento no esta regido por nada, en este ejemplo se envia velocidad y recivimos o modificamos la posicion de la llanta, esto deacuerdo a la derivada de la velocidad ya que no se tiene un control entre la velocidad y posicion.
