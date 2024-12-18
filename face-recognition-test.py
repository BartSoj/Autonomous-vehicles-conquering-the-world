import face_recognition_models
import face_recognition
import os

# Preload known celebrity encodings and names
KNOWN_IMAGES = [
    ("Mark Zuckerberg", "known_celebrities/mark.jpg"),
    ("Elon Musk", "known_celebrities/elon.jpg"),
    ("Linus Torvalds", "known_celebrities/linus.jpg"),
    ("Bill Gates", "known_celebrities/bill.jpg")
]

known_encodings = []
known_names = []

for name, image_path in KNOWN_IMAGES:
    image = face_recognition.load_image_file(image_path)
    encoding = face_recognition.face_encodings(image)[0]
    known_encodings.append(encoding)
    known_names.append(name)


def recognize_face(target_image_path):
    """Recognize the face in the given image against known encodings."""
    # Load and encode the target image
    target_image = face_recognition.load_image_file(target_image_path)
    target_encoding = face_recognition.face_encodings(target_image)

    if target_encoding:
        # Compare against known encodings
        distances = face_recognition.face_distance(known_encodings, target_encoding[0])
        best_match_index = distances.argmin()
        print(f"recognized {known_names[best_match_index]} with distance {distances[best_match_index]}")
    else:
        print("no face detected in the provided image.")


if __name__ == "__main__":
    test_dir = "test_celebrities"
    for target_image_name in os.listdir(test_dir):
        print(f"{target_image_name}: ", end="")
        target_image_path = os.path.join(test_dir, target_image_name)
        if os.path.exists(target_image_path):
            recognize_face(target_image_path)
        else:
            print(f"Error: The file {target_image_path} does not exist.")
