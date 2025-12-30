import json
import os
import tempfile
import time
import shutil

class StateFile:
    """
    Gère la lecture et l'écriture atomique d'un fichier d'état JSON.
    Garantit que les lecteurs ne voient jamais un fichier partiel.
    """
    def __init__(self, filepath):
        self.filepath = filepath
        self.last_read_time = 0
        self.cached_state = None

    def write(self, state):
        """
        Ecrit l'état de manière atomique :
        1. Ecrit dans un fichier temporaire
        2. Renomme le fichier temporaire (opération atomique sur Linux/POSIX)
        """
        # Creer un fichier temporaire dans le meme repertoire (important pour le rename atomique)
        directory = os.path.dirname(self.filepath)
        if not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            
        fd, temp_path = tempfile.mkstemp(dir=directory, text=True)
        try:
            with os.fdopen(fd, 'w') as f:
                json.dump(state, f)
            
            # Atomic replace
            os.replace(temp_path, self.filepath)
        except Exception as e:
            if os.path.exists(temp_path):
                os.remove(temp_path)
            raise e

    def read(self):
        """
        Lit l'état courant. Retourne None si le fichier n'existe pas ou est corrompu (rare avec atomic write).
        """
        if not os.path.exists(self.filepath):
            return None
            
        try:
            with open(self.filepath, 'r') as f:
                return json.load(f)
        except json.JSONDecodeError:
            # Peut arriver si on lit pile pendant le très court instant du swap (très rare)
            # ou si le disque est plein.
            print(f"[StateFile] Warning: JSON Decode Error on {self.filepath}")
            return None
