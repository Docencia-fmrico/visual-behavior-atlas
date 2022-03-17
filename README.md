[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-f059dc9a6f8d3a56e377f745f24479a46679e63a5d9fe6f495e02850cd0d8118.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7213008&assignment_repo_type=AssignmentRepo)
# Visual-behaviors Team Atlas


## Objetivo de la práctica

La práctica se compone de tres partes:

1. Seguimiento visual de la pelota: El robot debe seguir una pelota. El robot debe avanzar hasta estar a un metro, aproximadamente, de la pelota. A partir de ahí, seguira orientandose hacia ella, y tratando de mantener la distancia de un metro incluso si la pelota se mueve hacia el robot. Se usará:
   1.  Un filtro de color en HSV
   2.  Una estimación de la distancia al objeto filtrado, ya sea con PointCloud o con la imagen de profundidad.
   3.  Behavior Trees para programar la lógica de control.
   4.  PIDs para la orientación y la distancia.
2. Seguimiento visual de una persona: Similar al punto anterior, pero detectando a la persona con darket_ros.
3. Comportamiento mixto: El robot debe seguir tanto a las personas como a las pelotas que perciba con la cámara, teniendo prioridad la pelota.


## Follow Ball

 El nodo para la pelota consiste en dos acciones principales, detectar la pelota e ir hacia ella.
 Para realizarlo hemos seguido los siguentes pasos:
 
 **1.** Filtrado la pelota con hsv.
 
 **2.** Calculado la distancia a la que está la pelota mediante unas transaformadas.

 ![BT Follow Ball](https://github.com/Docencia-fmrico/visual-behavior-atlas/blob/main/Follow_Ball.jpeg)

 -[BT Follow Ball Video](https://github.com/Docencia-fmrico/visual-behavior-atlas/blob/main/Follow_Ball.mp4)

## Follow Person

 El nodo para la pelota consiste en dos acciones principales, detectar la persona e ir hacia ella.
 Para realizarlo hemos seguido los siguentes pasos:
 
 **1.** Utilizar el bounding box y filtrar que solo se quede con el id persona.
 
 **2.** Obtenemos la distancia de la persona con el mensaje del bounding box .


 ![BT Follow Person](https://github.com/Docencia-fmrico/visual-behavior-atlas/blob/main/Follow_Person.PNG)
 
 -[BT Follow Person Video](https://github.com/Docencia-fmrico/visual-behavior-atlas/blob/main/Follow_Ball.mp4)
 
 
