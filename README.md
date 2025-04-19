```markdown
# Montre Connectée - Projet Module Systèmes Embarqués Communicants

## Description du Projet

Ce projet documente le développement d'une **montre connectée basée sur le microcontrôleur nrf5340**, s'inspirant du projet open-source ZSWatch. Il s'inscrit dans le cadre du module Systèmes Embarqués Communicants du Master 1 TDSI - parcours Objets Connectés de l'Université de Poitiers. L'objectif principal était de concevoir et d'implémenter une montre connectée fonctionnelle intégrant diverses technologies, allant de la gestion des périphériques matériels à l'interaction utilisateur.

Le projet a permis d'intégrer des **capteurs environnementaux et de mouvement**, une **communication Bluetooth Low Energy**, une **interface graphique tactile** et une gestion précise du temps via un **module RTC dédié**. L'architecture logicielle repose sur le **système d'exploitation temps réel Zephyr**.

## Fonctionnalités Principales

La montre connectée implémente les fonctionnalités suivantes:

*   **Interface Utilisateur Graphique (GUI)**: Développée avec le framework **LVGL** sur un écran tactile 320x240. La conception graphique a été réalisée avec **Squareline Studio**.
*   **Capteurs de Mouvement et d'Orientation (IMU)**: Intégration et exploitation des capteurs **LSM6DSO** (accéléromètre et gyroscope) et **LIS2MDL** (magnétomètre) présents sur le shield ST IKS01A3.
*   **Capteurs Environnementaux**: Mesure de la température et de l'humidité via le capteur **HTS221** intégré au shield IKS01A3.
*   **Module Horloge Temps Réel (RTC)**: Gestion précise du temps grâce au module externe **RV-8263-C8**. Inclut une fonctionnalité de **chronomètre**.
*   **Communication Bluetooth Low Energy (BLE)**: Échange de données avec un smartphone, incluant la définition de services et caractéristiques **GATT** standard (ESS) et personnalisés pour les données des capteurs. Testé avec **nRF Connect for Mobile**.
*   **Détection de Pas (Podomètre)**: Détection des pas basée sur les données de l'accéléromètre avec estimation de la distance parcourue.
*   **Indication d'Orientation (Boussole)**: Calcul de l'orientation angulaire et conversion en direction cardinale à partir des données du magnétomètre, de l'accéléromètre et du gyroscope.

## Matériel Utilisé

*   **Microcontrôleur**: nrf5340.
*   **Shield Capteurs**: ST IKS01A3 (intégrant LSM6DSO, LIS2MDL, HTS221).
*   **Écran Tactile**: Adafruit 2.8" TFT Touch Shield v2.
*   **Module RTC**: RV-8263-C8 de Micro Crystal.
*   **Carte de Développement**: [Il faudrait spécifier ici la carte nRF5340 utilisée, par exemple nRF5340 DK].

## Logiciel Utilisé

*   **Système d'Exploitation Temps Réel (RTOS)**: Zephyr Project.
*   **Environnement de Développement (SDK)**: nRF Connect SDK de Nordic Semiconductor.
*   **Langage de Programmation**: C.
*   **Interface Graphique**: LVGL (Light and Versatile Graphics Library).
*   **Outil de Conception Graphique**: Squareline Studio.
*   **IDE (Environnement de Développement Intégré)**: Visual Studio Code (avec extensions Zephyr Tools).
*   **Outil de Programmation/Débogage**: Interface SWD via une sonde J-Link (ou intégrée à la carte de développement). Commande `west flash` ou interface de l'IDE.
*   **Driver Touchscreen**: **Un driver spécifique pour l'écran tactile a été développé par notre professeur (lien :https://github.com/hboeglen/squareline_studio_nrf5340dk_plugins)**. Ce driver est intégré à l'environnement Zephyr pour la gestion des entrées tactiles.

## Démarrage

Pour compiler et flasher le code sur la carte nRF5340, vous aurez besoin de configurer l'environnement de développement nRF Connect SDK. Référez-vous à la documentation de Nordic Semiconductor pour les étapes d'installation. Une fois l'environnement configuré, vous pourrez utiliser les commandes `west build` et `west flash` pour compiler le projet et le téléverser sur la carte. La configuration spécifique des capteurs et périphériques est gérée via les fichiers DeviceTree (`.overlay`) et Kconfig (`prj.conf`).

## Utilisation

Une fois le firmware flashé sur la montre, l'écran tactile permet de naviguer entre différentes vues affichant l'heure, les données des capteurs (température, humidité, accélération, orientation), le nombre de pas et un chronomètre. La montre est également capable de communiquer via Bluetooth Low Energy avec une application mobile compatible (par exemple, nRF Connect) pour afficher les données des capteurs en temps réel.

## Équipe Projet

*   **[Maxime]** : https://github.com/M0x3m
*   **[Moi]**
*   **[Rayane DJENADOU]** 


```
