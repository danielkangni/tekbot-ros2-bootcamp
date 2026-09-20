"""Test 1 - Tekbot Robotics Challenges 2024 : hiérarchie de classes pour un Robot.

Robot (classe de base)
 ├── WheeledRobot  : robot à roues
 └── LeggedRobot   : robot à jambes
"""


class Robot:
    """Classe de base représentant un robot générique."""

    def __init__(self, nom, batterie=100.0, vitesse=1.0):
        self._nom = nom
        self._batterie = 100.0
        self._vitesse = 1.0
        self.batterie = batterie
        self.vitesse = vitesse

    @property
    def nom(self):
        return self._nom

    @nom.setter
    def nom(self, valeur):
        if not valeur or not isinstance(valeur, str):
            raise ValueError("Le nom doit être une chaîne non vide.")
        self._nom = valeur

    @property
    def batterie(self):
        """Niveau de batterie en pourcentage (0 à 100)."""
        return self._batterie

    @batterie.setter
    def batterie(self, valeur):
        if not 0 <= valeur <= 100:
            raise ValueError("La batterie doit être comprise entre 0 et 100.")
        self._batterie = float(valeur)

    @property
    def vitesse(self):
        """Vitesse en m/s (positive)."""
        return self._vitesse

    @vitesse.setter
    def vitesse(self, valeur):
        if valeur <= 0:
            raise ValueError("La vitesse doit être strictement positive.")
        self._vitesse = float(valeur)

    # ------ Comportement ----------
    def _consommer(self, quantite):
        """Retire de la batterie ; retourne False si elle est insuffisante."""
        if self._batterie < quantite:
            print(f"{self._nom} : batterie insuffisante.")
            return False
        self._batterie -= quantite
        return True

    def move(self, distance=1.0):
        """Déplacement générique. Redéfinie dans les sous-classes."""
        if self._consommer(distance * 1.0):
            print(f"{self._nom} se déplace de {distance} m.")

        # on passe par les setters pour bénéficier de la validation
    def __str__(self):
        return (f"{type(self).__name__}(nom={self._nom}, "
                f"batterie={self._batterie:.1f}%, vitesse={self._vitesse} m/s)")


class WheeledRobot(Robot):
    """Robot à roues : déplacement rapide et économe."""

    def __init__(self, nom, nb_roues=4, **kwargs):
        super().__init__(nom, **kwargs)
        self._nb_roues = 4
        # on passe par les setters pour bénéficier de la validation
        self.nb_roues = nb_roues

    @property
    def nb_roues(self):
        return self._nb_roues

    @nb_roues.setter
    def nb_roues(self, valeur):
        if valeur < 2:
            raise ValueError("Un robot à roues a au moins 2 roues.")
        self._nb_roues = int(valeur)

    def move(self, distance=1.0):
        if self._consommer(distance * 0.5):
            print(f"{self._nom} roule sur {self._nb_roues} roues "
                  f"de {distance} m à {self._vitesse} m/s.")


class LeggedRobot(Robot):
    """Robot à jambes : déplacement lent et coûteux en énergie."""

    def __init__(self, nom, nb_jambes=2, **kwargs):
        super().__init__(nom, **kwargs)
        self._nb_jambes = 2
        self.nb_jambes = nb_jambes

    @property
    def nb_jambes(self):
        return self._nb_jambes

    @nb_jambes.setter
    def nb_jambes(self, valeur):
        if valeur < 2:
            raise ValueError("Un robot à jambes a au moins 2 jambes.")
        self._nb_jambes = int(valeur)

    def move(self, distance=1.0):
        pas = int(distance / 0.3) or 1
        if self._consommer(distance * 2.0):
            print(f"{self._nom} marche sur {self._nb_jambes} jambes : "
                  f"{pas} pas pour {distance} m.")


if __name__ == "__main__":
    robots = [
        Robot("Generic"),
        WheeledRobot("Rover", nb_roues=6, vitesse=2.5),
        LeggedRobot("Spider", nb_jambes=6, vitesse=0.5),
    ]
    for r in robots:
        print(r)
        r.move(3)
        print(f"  -> batterie restante : {r.batterie:.1f}%\n")

    try:
        robots[0].batterie = 150
    except ValueError as e:
        print("Erreur attendue :", e)
