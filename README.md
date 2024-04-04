# ProjetInfoScientifique

Ceci est un projet réalisé pour l'UE "Projet d'informatique scientifique" dans le cadre de la licence 3 Informatique-Mathématiques CMI OPTIM à l'université de Nantes. \
Il s'agit d'implémentation d'algorithmes de plus court chemin (Floodfill, Dijsktyra, A*) en julia

### Packages
Utilisation des packages `DataStructures` (pour utiliser une implémentation d'un tas min) ;\
`Printf` (un affichage précis des temps)\
`PlotlyJS` pour un affichage graphique des resultats

### Utilisation :

les fonctions `algoFloodFill`, `algoDijsktra`, `algoAstar` et `algoWAstar` peuvent être utulisées individuellement \
La fonction `testAlgos` permet de tester ces implementations, et de donner un affichage du temps d'éxecution et du nombre d'états visités.
La fonction `testWAstar` permet de comparer differentes valeur de w pour Astar (valeurs predefinies).
