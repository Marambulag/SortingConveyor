import serial

def byte_to_bin(byte):
    return format(byte, '08b')

def main():
    puerto = "COM8"  # Ajusta al puerto correspondiente si cambió
    baudrate = 115200

    with serial.Serial(puerto, baudrate, timeout=1) as ser:
        print("Eco UART – 0–3.")
        while True:
            entrada = input(">> ")
            if entrada.lower() == "q":
                break
            if entrada not in ["0", "1", "2", "3"]:
                print("Usa 0, 1, 2, 3 o q para salir.")
                continue

            valor = int(entrada)
            ser.write(bytes([valor]))
            print(f"Enviado: {valor:02X} (bin: {byte_to_bin(valor)})")

            eco = ser.read(1)
            if eco:
                eco_val = eco[0]
                print(f"Echo recibido: {eco_val:02X} (bin: {byte_to_bin(eco_val)})")
            else:
                print("No se recibió eco")

if __name__ == "__main__":
    main()
