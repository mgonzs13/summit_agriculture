"""
FLOR – Fichas con Localización, Observaciones y Referencia visual
"""

# Importaciones de biblioteca estándar
import base64
from io import BytesIO

# Importaciones de bibliotecas externas
import streamlit as st
import pandas as pd
import folium
from streamlit_folium import st_folium
from folium import IFrame, plugins
from folium.plugins import MiniMap
from PIL import Image
import requests


def get_image_from_url(url):
    """Obtiene una imagen desde una URL."""
    try:
        response = requests.get(url)
        return Image.open(BytesIO(response.content))
    except:
        return None


def image_to_base64(img):
    """Convierte una imagen a formato base64 para incluir en folium."""
    buffered = BytesIO()
    img.save(buffered, format="PNG")
    return base64.b64encode(buffered.getvalue()).decode()


def crear_mapa_base(use_super_zoom=True):
    """Crea un mapa base con múltiples capas."""
    mapa = folium.Map(zoom_start=2, max_zoom=30 if use_super_zoom else 25)
    
    # Capa satelital ESRI
    satellite_tiles = ('https://server.arcgisonline.com/ArcGIS/rest/'
                       'services/World_Imagery/MapServer/tile/{z}/{y}/{x}')
    satellite_attr = ('Tiles &copy; Esri &mdash; Source: Esri, i-cubed, '
                     'USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, '
                     'IGN, IGP, UPR-EGP, and the GIS User Community')
    folium.TileLayer(
        tiles=satellite_tiles,
        attr=satellite_attr,
        name='Satelital ESRI',
        max_zoom=23
    ).add_to(mapa)
    
    # Capa de Google Satelital
    folium.TileLayer(
        tiles='https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}',
        attr='Google Satellite',
        name='Satelital Google',
        max_zoom=22
    ).add_to(mapa)
    
    # Capa de Google Híbrido
    folium.TileLayer(
        tiles='https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}',
        attr='Google Hybrid',
        name='Google Híbrido',
        max_zoom=22
    ).add_to(mapa)
    
    # Capa de OpenStreetMap
    folium.TileLayer(
        tiles='OpenStreetMap',
        name='OpenStreetMap',
        max_zoom=19
    ).add_to(mapa)
    
    return mapa


def agregar_marcadores(mapa, df, selected_plant, use_super_zoom):
    """Agrega marcadores de plantas al mapa."""
    for _, row in df.iterrows():
        # Crear popup con información básica e imagen
        html = f"""
            <div style="width:220px; font-size:14px;">
                <b>ID:</b> {row['id']}<br>
                <b>Lat:</b> {row['latitud']}<br>
                <b>Lon:</b> {row['longitud']}<br><br>
                <img src="{row['imagen']}" width="200px" style="border-radius:8px;">
            </div>
        """
        popup = folium.Popup(IFrame(html, width=230, height=260), max_width=230)

        # Destacar la planta seleccionada con color diferente
        icon_color = "red" if selected_plant != "Ver todas" and str(row['id']) == str(selected_plant) else "green"
        
        # Añadir marcador normal
        folium.Marker(
            location=[row["latitud"], row["longitud"]],
            popup=popup,
            icon=folium.Icon(color=icon_color, icon="leaf", prefix="fa")
        ).add_to(mapa)
        
        # Añadir área de super zoom si está activado y es la planta seleccionada
        if use_super_zoom and str(selected_plant) == str(row['id']):
            lat, lon = row["latitud"], row["longitud"]
            
            # Añadir un marcador especial para super zoom
            folium.CircleMarker(
                location=[lat, lon],
                radius=10,
                color='blue',
                fill=True,
                fill_color='blue',
                fill_opacity=0.3,
                popup=folium.Popup("Área de super zoom - Puedes ampliar esta área más allá del límite normal", max_width=200)
            ).add_to(mapa)


def agregar_plugins_al_mapa(mapa):
    """Agrega plugins de utilidad al mapa."""
    # Control de capas
    folium.LayerControl().add_to(mapa)
    
    # Botón de pantalla completa
    plugins.Fullscreen().add_to(mapa)
    
    # Minimapa para referencia
    minimap = MiniMap(toggle_display=True)
    mapa.add_child(minimap)
    
    # Plugin de medición para escala
    plugins.MeasureControl(position='bottomleft', primary_length_unit='meters').add_to(mapa)


def centrar_mapa(mapa, df, selected_plant, coordenadas):
    """Centra el mapa según la selección del usuario."""
    if selected_plant != "Ver todas":
        selected_row = df[df['id'] == selected_plant].iloc[0]
        mapa.location = [selected_row['latitud'], selected_row['longitud']]
        mapa.zoom_start = 19  # Zoom cercano para la planta seleccionada
        
        # Forzar el zoom a la planta seleccionada
        mapa.fit_bounds([[selected_row['latitud'], selected_row['longitud']], 
                       [selected_row['latitud'], selected_row['longitud']]])
    else:
        # Centrar el mapa en todos los puntos
        mapa.fit_bounds(coordenadas)


def mostrar_vista_ampliada(df, selected_plant):
    """Muestra una vista ampliada de la imagen de la planta seleccionada."""
    if selected_plant != "Ver todas":
        selected_row = df[df['id'] == selected_plant].iloc[0]
        st.subheader(f"🔍 Vista ampliada de la planta: {selected_plant}")
        st.image(selected_row['imagen'], caption=f"ID: {selected_plant}", width=400)
        st.write("Puedes hacer zoom directamente en esta imagen usando los controles del navegador (Ctrl + rueda del ratón)")


def main():
    """Función principal de la aplicación."""
    st.set_page_config(page_title="FLOR - Mapa de Plantas", layout="wide")
    st.title("🌸 FLOR - Visualizador de Plantas con GPS e Imagen")

    uploaded_file = st.file_uploader("📁 Sube tu archivo CSV con ID, coordenadas e imagen", type="csv")

    if uploaded_file is not None:
        df = pd.read_csv(uploaded_file)

        # Validar columnas requeridas
        required_columns = {"id", "latitud", "longitud", "imagen"}
        if not required_columns.issubset(df.columns):
            st.error(f"❌ El archivo debe contener las columnas: {', '.join(required_columns)}")
            return
            
        # Obtener lista de coordenadas
        coordenadas = df[["latitud", "longitud"]].values.tolist()
        
        # Opciones de configuración
        use_super_zoom = st.checkbox("🔍+ Activar Super Zoom (para ver detalles extremos)", value=True)
        
        # Crear mapa base
        mapa = crear_mapa_base(use_super_zoom)
        
        # Selector de planta específica
        plant_ids = df['id'].tolist()
        selected_plant = st.selectbox("🔍 Hacer zoom a una planta específica:", 
                                     ["Ver todas"] + plant_ids)
        
        # Agregar marcadores al mapa
        agregar_marcadores(mapa, df, selected_plant, use_super_zoom)
        
        # Mostrar nota sobre super zoom si corresponde
        if use_super_zoom and selected_plant != "Ver todas":
            st.info("🔎 Modo Super Zoom: Puedes hacer zoom más allá del límite normal en el área azul alrededor de la planta seleccionada")
        
        # Agregar plugins al mapa
        agregar_plugins_al_mapa(mapa)
        
        # Centrar el mapa según la selección
        centrar_mapa(mapa, df, selected_plant, coordenadas)

        # Mostrar mapa en Streamlit
        map_data = st_folium(mapa, width=950, height=650)
        
        # Mostrar vista ampliada de la imagen seleccionada
        mostrar_vista_ampliada(df, selected_plant)
    else:
        st.info("👆 Sube un archivo CSV con columnas: id, latitud, longitud, imagen.")


if __name__ == "__main__":
    main()
