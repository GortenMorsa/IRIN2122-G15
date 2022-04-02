# Trabajo O1 IRIN 2021-2022
## Localización del proyecto
* ~Parte reactiva y con comportamientos : ArqComportamiento~
* ~Ampliación con resolución de mapas: ResolucionMapas~
Todo en main, si se quiere observar la parte de comportamientos únicamente, comentar las funciones relacionadas con resolución de mapas y cambiar el número de comportamientos

## Comandos
Es recomendable descargar el proyecto completo desde GitHub y ejecutar en /irsim2022:

```
$ ./bootstrap.sh
$ ./configure
$ make
```

Se ha usado como base el test 'iri3', por lo tanto, para ejecutar el proyecto usar en un terminal:

```
$ ./irsim -E 32 -p paramFiles/iri3Param.txt 
```
## Condiciones del robot y parámetros bajo los que se ha probado:
* Velocidad del robot <= ~150~ 300 con PROXIMITY_THRESHOLD <= 0.4 y módulo de ObstacleAvoidance = 1.0 (en mapas complejos evita problemas en cuanto a giros y colisiones, pero tener cuidado si se intenta hacer pasar el robot entre un pasillo muy estrecho)
* Luces azules lo más próximas al centro de su respectiva zona gris (evita conflictos respecto a la activación de ~Forage~ PickUp, puesto que necesita recibir suficiente iluminación y a la vez estar dentro de una zona gris)
* Zonas grises lo suficientemente separadas (ya que el apagar una luz no implica la destrucción del objeto, es posible "reapagar" una luz apagada y que ePuck interprete que ha alcanzado una zona nueva)
* Usar precisión 30 para la resolución de mapas, mejora considerablemente la precisión de GoGoal
