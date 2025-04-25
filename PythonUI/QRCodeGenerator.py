import qrcode
import json

# Condensed boarding pass data
boarding_pass = {
    "name": "Terrier",
    "flight": "BU2025",
    "from": "BOS",
    "to": "MC0",
    "dep_time": "2:15PM",
    "terminal": "A",
    "gate": "3",
}

# Convert to JSON string with no extra whitespace
boarding_pass_str = json.dumps(boarding_pass, separators=(",", ":"))

# Generate the QR code using the simple factory method
qr_image = qrcode.make(boarding_pass_str)
qr_image.show()
