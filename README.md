
# Cible de fléchettes interactive à géolocalisation d’impact et interface temps réel

Projet 5A AE-SE Double Diplôme INSA-TSM

L'objectif est de transformer une cible de fléchettes conventionnelle en un système interactif, connecté et temps réel. Le système doit être capable de détecter mécaniquement le point d'impact d'une fléchette, calculer ses coordonnées spatiales, puis de transmettre ces données à une application distante pour gérer le score et l'affichage graphique interactif.
## Architecture du dépôt

```
dartboard-connect/
├── README.md
├── docs/                      # Rapport, spécifications
├── hardware/                  # Cartes & Électronique
├── firmware/                  # Code Embarqué STM32 (C/C++)
└── software/                  # Code PC (Traitement & IHM)
```