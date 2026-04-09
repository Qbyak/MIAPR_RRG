# Implementacja algorytmu RRG jako planera ścieżki ruchu dla robota mobilnego Turtlebot3 w Navigation Stack
## Głównym celem projektu jest implementacja algorytmu RRG jako autorskiego planera ścieżki w języku C++ oraz jego integracja z systemem Nav2. W ramach prac zrealizowano następujące zadania:
• Uruchomienie symulacji robota Turtlebot i test nawigacji na wczytanej mapie zajętości.
• Podmiana domyślnego planera w stosie nawigacyjnym Nav2 na autorski moduł (plugin) oparty
na RRG.
• Weryfikacja działania zaimplementowanego algorytmu na modelu robota Turtlebot3 w środo-
wisku symulacyjnym.
• Przeprowadzenie eksperymentów w dwóch różnych topologicznie środowiskach mapy zajętości.
• Porównanie wydajności RRG z domyślnym planerem pod kątem średniej długości wygenero-
wanej ścieżki oraz czasu planowania
