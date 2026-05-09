import numpy as np

def test_database_persistence(tmp_path):
    from robertito_face_recognition.face_db import FaceDatabase

    db_path = tmp_path / "faces.json"
    db = FaceDatabase(str(db_path))
    embedding = np.linspace(0.0, 1.0, num=128)
    db.update_person("test_user", embedding)

    reloaded = FaceDatabase(str(db_path))
    entries = reloaded.entries()
    assert "test_user" in entries
    assert np.allclose(entries["test_user"], embedding)


def test_matching_threshold(tmp_path):
    from robertito_face_recognition.face_db import FaceDatabase

    db_path = tmp_path / "faces.json"
    db = FaceDatabase(str(db_path))
    db.update_person("alice", np.zeros(128))
    db.update_person("bob", np.ones(128))

    name, distance = db.find_best_match(np.zeros(128), threshold=0.5)
    assert name == "alice"
    assert distance is not None and distance <= 0.1

    name, distance = db.find_best_match(np.ones(128) * 0.5, threshold=0.1)
    assert name is None
    assert distance is not None
