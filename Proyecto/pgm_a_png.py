import struct
import sys
import zlib


def leer_pgm(nombre):
    with open(nombre, 'rb') as f:
        datos = f.read()
    # cabecera: P5, ancho, alto, maximo (puede haber comentarios con #)
    campos = []
    i = 0
    while len(campos) < 4:
        while datos[i:i + 1].isspace():
            i += 1
        if datos[i:i + 1] == b'#':
            while datos[i:i + 1] != b'\n':
                i += 1
            continue
        inicio = i
        while not datos[i:i + 1].isspace():
            i += 1
        campos.append(datos[inicio:i])
    i += 1
    if campos[0] != b'P5':
        raise ValueError('Solo se admite PGM binario (P5)')
    ancho = int(campos[1])
    alto = int(campos[2])
    return ancho, alto, datos[i:i + ancho * alto]


def escribir_png(nombre, ancho, alto, pixeles):
    def trozo(tipo, contenido):
        cuerpo = tipo + contenido
        return struct.pack('>I', len(contenido)) + cuerpo + struct.pack('>I', zlib.crc32(cuerpo))

    filas = b''.join(b'\x00' + pixeles[y * ancho:(y + 1) * ancho] for y in range(alto))
    png = b'\x89PNG\r\n\x1a\n'
    png += trozo(b'IHDR', struct.pack('>IIBBBBB', ancho, alto, 8, 0, 0, 0, 0))
    png += trozo(b'IDAT', zlib.compress(filas))
    png += trozo(b'IEND', b'')
    with open(nombre, 'wb') as f:
        f.write(png)


if __name__ == '__main__':
    entrada = sys.argv[1] if len(sys.argv) > 1 else 'mapa_robot.pgm'
    salida = sys.argv[2] if len(sys.argv) > 2 else entrada.rsplit('.', 1)[0] + '.png'
    ancho, alto, pixeles = leer_pgm(entrada)
    escribir_png(salida, ancho, alto, pixeles)
    print(f'{entrada} ({ancho}x{alto}) -> {salida}')
