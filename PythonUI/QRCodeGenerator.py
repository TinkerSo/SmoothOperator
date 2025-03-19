import qrcode
import json

# Condensed boarding pass data
boarding_pass = {
    "name": "SmoothOperator",
    "flight": "AA1234",
    "from": "BOS",
    "to": "LAX",
    "dep_time": "8:30PM",
    "terminal": "B",
    "gate": "12",
}

# Convert to JSON string
boarding_pass_str = json.dumps(boarding_pass, separators=(",", ":"))

# Generate a simple QR code with minimal complexity
qr = qrcode.QRCode(
    version=1,  # Smallest version
    error_correction=qrcode.constants.ERROR_CORRECT_L,  # Lowest error correction for simplicity
    box_size=6,  # Adjusts size of individual squares
    border=2  # Reduces border thickness
)
qr.add_data(boarding_pass_str)
qr.make(fit=True)

# Create and save the QR code image
qr_image = qr.make_image(fill="black", back_color="white")
qr_image.show()
