# Trabajo O1 IRIN 2021-2022
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
* Velocidad del robot <= 150 (en mapas complejos evita problemas en cuanto a giros y colisiones)
* Luces azules lo más próximas al centro de su respectiva zona gris (evita conflictos respecto a la activación de Forage, puesto que necesita recibir suficiente iluminación y a la vez estar dentro de una zona gris)
* Zonas grises lo suficientemente separas (ya que el apagar una luz no implica la destrucción del objeto, es posible "reapagar" una luz apagada y que ePuck interprete que ha alcanzado una zona nueva)
