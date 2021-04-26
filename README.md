Versione ridotta del POMCP originale, dato un belief computa l'azione da scegliere.
Pensato per essere usato solo sul problema velocity regulation, in un rettangolo 3x5.

Per usarlo, specificare la distribuzione sugli 81 stati (opzione --belief), la posizione (--pos),
la potenza del numero di particelle usate (--mindouble, per esempio --mindouble 15 usa 2^15 particelle),
ed eventualmente il reward range (opzione --W, 52 di default).

per compilare:
    mkdir build && cd build
    cmake ..
    make

