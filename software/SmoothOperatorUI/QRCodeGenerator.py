import qrcode
import json

# Condensed boarding pass data
boarding_pass = {
    "name": "Professor Michael Hirsch",
    "flight": "AA1234",
    "from": "BOS",
    "to": "LAX",
    "dep_time": "8:30PM",
    "terminal": "B",
    "gate": "12",
}

# Convert to JSON string with no extra whitespace
boarding_pass_str = json.dumps(boarding_pass, separators=(",", ":"))

# Generate the QR code using the simple factory method
qr_image = qrcode.make(boarding_pass_str)
qr_image.show()
