"""
Utilidad para generar trayectorias de líneas paralelas a partir de coordenadas GPS.

Este módulo permite crear patrones de movimiento dentro de polígonos definidos por 
coordenadas GPS, útil para crear rutas de cobertura total en aplicaciones de 
agricultura, inspección o mapeo.
"""

# Importaciones de biblioteca estándar
import math
import csv

# Importaciones de bibliotecas externas
import yaml
from shapely.geometry import Polygon, LineString
from shapely.affinity import translate
from geopy.distance import geodesic


def cargar_puntos_desde_yaml(ruta_archivo):
    """
    Carga puntos GPS desde un archivo YAML.
    
    Args:
        ruta_archivo: Ruta al archivo YAML con los waypoints
        
    Returns:
        Lista de tuplas (latitud, longitud)
    """
    with open(ruta_archivo, 'r') as f:
        data = yaml.safe_load(f)
    waypoints = data['waypoints']
    return [(wp['latitude'], wp['longitude']) for wp in waypoints]


def guardar_puntos_en_yaml(lineas, ruta_archivo):
    """
    Guarda puntos GPS en formato YAML.
    
    Args:
        lineas: Lista de listas de puntos GPS [(lat, lon), ...]
        ruta_archivo: Ruta donde guardar el archivo YAML
    """
    waypoints = []
    
    for linea in lineas:
        for lat, lon in linea:
            waypoints.append({
                'latitude': lat,
                'longitude': lon,
                'yaw': 0.0
            })
    
    data = {'waypoints': waypoints}
    
    with open(ruta_archivo, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)
    
    print(f"Puntos guardados en formato YAML: {ruta_archivo}")


def guardar_puntos_en_csv(lineas, ruta_archivo):
    """
    Guarda puntos GPS en formato CSV.
    
    Args:
        lineas: Lista de listas de puntos GPS [(lat, lon), ...]
        ruta_archivo: Ruta donde guardar el archivo CSV
    """
    rows = [['id', 'latitud', 'longitud', 'imagen', 'confianza', 'persona']]
    
    for i, linea in enumerate(lineas):
        for j, (lat, lon) in enumerate(linea):
            punto_id = f"linea{i+1}_{j+1}"
            rows.append([
                punto_id,
                f"{lat:.10f}",
                f"{lon:.10f}",
                f"{punto_id}.png",
                f"{0.7:.5f}",
                ""
            ])
    
    with open(ruta_archivo, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(rows)
    
    print(f"Puntos guardados en formato CSV: {ruta_archivo}")


def gps_to_cartesian(ref_point, gps_point):
    """
    Convierte coordenadas GPS a coordenadas cartesianas relativas.
    
    Args:
        ref_point: Punto de referencia (lat, lon)
        gps_point: Punto GPS a convertir (lat, lon)
        
    Returns:
        Tupla (x, y) en metros relativos al punto de referencia
    """
    lat_ref, lon_ref = ref_point
    lat, lon = gps_point
    
    dx = geodesic((lat_ref, lon_ref), (lat_ref, lon)).meters
    dy = geodesic((lat_ref, lon_ref), (lat, lon_ref)).meters
    
    if lon < lon_ref:
        dx = -dx
    if lat < lat_ref:
        dy = -dy
        
    return (dx, dy)


def cartesian_to_gps(ref_point, x, y):
    """
    Convierte coordenadas cartesianas relativas a coordenadas GPS.
    
    Args:
        ref_point: Punto de referencia GPS (lat, lon)
        x: Coordenada x en metros
        y: Coordenada y en metros
        
    Returns:
        Tupla (latitud, longitud)
    """
    lat_ref, lon_ref = ref_point
    new_lat = geodesic(meters=y).destination((lat_ref, lon_ref), 0).latitude
    new_lon = geodesic(meters=x).destination((lat_ref, lon_ref), 90).longitude
    return (new_lat, new_lon)


def _encontrar_lados_poligono(puntos_xy):
    """
    Encuentra los lados más largo y más corto de un polígono.
    
    Args:
        puntos_xy: Lista de puntos cartesianos del polígono
        
    Returns:
        Tupla (lado_mas_largo, lado_mas_corto) como LineStrings
    """
    max_len = 0
    min_len = float('inf')
    lado_mas_largo = None
    lado_mas_corto = None
    
    for i in range(len(puntos_xy)):
        p1 = puntos_xy[i]
        p2 = puntos_xy[(i + 1) % len(puntos_xy)]
        length = math.dist(p1, p2)
        
        if length > max_len:
            max_len = length
            lado_mas_largo = LineString([p1, p2])
        
        if length < min_len and length > 0:
            min_len = length
            lado_mas_corto = LineString([p1, p2])
            
    return lado_mas_largo, lado_mas_corto


def _procesar_interseccion(interseccion):
    """
    Procesa una intersección geométrica para obtener una LineString válida.
    
    Args:
        interseccion: Objeto geométrico de intersección
        
    Returns:
        LineString o None si la intersección no es válida
    """
    if interseccion.is_empty or not isinstance(interseccion, LineString) or interseccion.length < 0.1:
        return None
        
    # Si la intersección produce múltiples líneas, tomamos la más larga
    if hasattr(interseccion, 'geoms'):
        longest = None
        max_length = 0
        for geom in interseccion.geoms:
            if isinstance(geom, LineString) and geom.length > max_length:
                longest = geom
                max_length = geom.length
        return longest
    
    return interseccion


def _crear_puntos_en_linea(linea, ref_point, densidad_puntos):
    """
    Crea puntos a lo largo de una línea con la densidad especificada.
    
    Args:
        linea: LineString donde generar puntos
        ref_point: Punto GPS de referencia
        densidad_puntos: Distancia entre puntos en metros
        
    Returns:
        Lista de puntos GPS [(lat, lon), ...]
    """
    puntos_linea = []
    num_puntos = max(1, int(linea.length / densidad_puntos))
    
    for i in range(num_puntos + 1):
        punto = linea.interpolate(i / num_puntos, normalized=True)
        latlon = cartesian_to_gps(ref_point, punto.x, punto.y)
        puntos_linea.append(latlon)
        
    return puntos_linea


def generar_lineas_paralelas(puntos_gps, distancia_entre_lineas=1.0, densidad_puntos=3.0, direccion="perpendicular_largo"):
    """
    Genera líneas paralelas dentro de un polígono definido por coordenadas GPS.
    
    Args:
        puntos_gps: Lista de coordenadas GPS que forman el polígono
        distancia_entre_lineas: Separación entre líneas (en metros)
        densidad_puntos: Distancia entre puntos en cada línea (en metros)
        direccion: Dirección de las líneas, opciones:
            * "perpendicular_largo": Perpendicular al lado más largo (default)
            * "paralelo_largo": Paralelo al lado más largo
            * "perpendicular_corto": Perpendicular al lado más corto
            * "paralelo_corto": Paralelo al lado más corto
            
    Returns:
        Lista de líneas, donde cada línea es una lista de puntos GPS
    """
    # Convertir puntos GPS a coordenadas cartesianas
    ref_point = puntos_gps[0]
    puntos_xy = [gps_to_cartesian(ref_point, p) for p in puntos_gps]
    poligono = Polygon(puntos_xy)

    # Encontrar los lados del polígono
    lado_mas_largo, lado_mas_corto = _encontrar_lados_poligono(puntos_xy)

    # Seleccionar la línea base según la dirección elegida
    perpendicular = True
    if direccion == "perpendicular_largo":
        base_line = lado_mas_largo
        perpendicular = True
    elif direccion == "paralelo_largo":
        base_line = lado_mas_largo
        perpendicular = False
    elif direccion == "perpendicular_corto":
        base_line = lado_mas_corto
        perpendicular = True
    elif direccion == "paralelo_corto":
        base_line = lado_mas_corto
        perpendicular = False
    else:
        base_line = lado_mas_largo
        print(f"Dirección '{direccion}' no válida. Usando 'perpendicular_largo'.")

    # Vector director de la línea base
    dx = base_line.coords[1][0] - base_line.coords[0][0]
    dy = base_line.coords[1][1] - base_line.coords[0][1]
    
    # Vector normal o paralelo según la opción elegida
    vector = (-dy, dx) if perpendicular else (dx, dy)
    
    # Normalizar el vector
    length_vector = math.hypot(*vector)
    unit_vector = (vector[0] / length_vector, vector[1] / length_vector)

    # Generar líneas paralelas en ambos sentidos
    lineas_con_posicion = []
    
    # Añadir la línea central primero (offset 0)
    inter = poligono.intersection(base_line)
    if not inter.is_empty and isinstance(inter, LineString):
        puntos_linea = _crear_puntos_en_linea(inter, ref_point, densidad_puntos)
        lineas_con_posicion.append((puntos_linea, 0))
    
    # Generar líneas en ambas direcciones
    for direction in [-1, 1]:
        offset = distancia_entre_lineas
        linea_valida = True
        
        while linea_valida:
            desplazamiento = (
                unit_vector[0] * offset * direction,
                unit_vector[1] * offset * direction
            )
            linea = translate(base_line, xoff=desplazamiento[0], yoff=desplazamiento[1])
            inter = _procesar_interseccion(poligono.intersection(linea))
            
            if inter is None:
                linea_valida = False
                continue
                
            puntos_linea = _crear_puntos_en_linea(inter, ref_point, densidad_puntos)
            lineas_con_posicion.append((puntos_linea, offset * direction))
            offset += distancia_entre_lineas
    
    # Ordenar las líneas según su posición (de menor a mayor)
    lineas_con_posicion.sort(key=lambda x: x[1])
    
    # Extraer solo las líneas ordenadas
    return [linea for linea, _ in lineas_con_posicion]


if __name__ == "__main__":
    """Ejemplo de uso del módulo"""
    archivo_yaml = "puntos.yaml"
    puntos_gps = cargar_puntos_desde_yaml(archivo_yaml)

    lineas = generar_lineas_paralelas(
        puntos_gps,
        distancia_entre_lineas=0.55,
        densidad_puntos=2.0,
        direccion="perpendicular_largo"
    )

    # Guardar resultados
    guardar_puntos_en_yaml(lineas, "trayectoria.yaml")
    guardar_puntos_en_csv(lineas, "trayectoria.csv")

    # Mostrar resultados
    for i, linea in enumerate(lineas):
        print(f"Línea {i+1}:")
        for j, punto in enumerate(linea):
            print(f"  - id: linea{i+1}_{j+1}, lat: {punto[0]:.8f}, lon: {punto[1]:.8f}")
