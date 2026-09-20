# Test 1 : Classe Robot (Python)

## Architecture

Une classe de base `Robot` et deux sous-classes, `WheeledRobot` (à roues) et `LeggedRobot` (à jambes).

### Diagramme de classes UML

```mermaid
classDiagram
    class Robot {
        -str _nom
        -float _batterie
        -float _vitesse
        +nom : str
        +batterie : float
        +vitesse : float
        #_consommer(quantite) bool
        +move(distance)
        +__str__() str
    }
    class WheeledRobot {
        -int _nb_roues
        +nb_roues : int
        +move(distance)
    }
    class LeggedRobot {
        -int _nb_jambes
        +nb_jambes : int
        +move(distance)
    }
    Robot <|-- WheeledRobot
    Robot <|-- LeggedRobot
```

### Diagramme de séquence : polymorphisme de `move()`

```mermaid
sequenceDiagram
    participant Main
    participant R as Robot
    participant W as WheeledRobot
    participant L as LeggedRobot
    Main->>R: move(3)
    R-->>Main: déplacement générique (coût 1.0/m)
    Main->>W: move(3)
    W-->>Main: roule (coût 0.5/m)
    Main->>L: move(3)
    L-->>Main: marche, compte les pas (coût 2.0/m)
```

## Concepts démontrés

| Concept | Où |
|---|---|
| **Encapsulation** | Attributs privés `_nom`, `_batterie`, `_vitesse`, `_nb_roues`, `_nb_jambes`, accessibles via des propriétés (getters/setters) avec validation (`ValueError`) |
| **Héritage** | `WheeledRobot` et `LeggedRobot` héritent de `Robot` (`super().__init__`) |
| **Polymorphisme** | `move()` est définie dans `Robot` puis redéfinie dans chaque sous-classe ; la boucle du `__main__` l'appelle sans connaître le type exact |
| **Méthode protégée** | `_consommer()` factorise la gestion de la batterie |

## Exécution

```bash
python3 class.py
```

## Extension

Pour ajouter un robot (par exemple `FlyingRobot`), il suffit d'hériter de `Robot` et de redéfinir `move()`. Le code appelant ne change pas.
