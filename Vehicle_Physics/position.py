from abc import abstractmethod
import numpy as np
from Utility.constants import EARTH_RADIUS

"""
ROUND EARTH ASSUMPTION
NEED TO CONVERT TO ECEF AND ADD ITERATION FOR EFEF -> GEODETIC

Contains: 
geodetic coordinates: lat.long.height [rad]
geocentric coodrinates (geo)xyz [m]
earth coordinates (earth)xyz [m]
"""
class Position():
    
    def __init__(self,geodetic,geocentric,earth):
        self._geodetic = np.asarray(geodetic,dtype=float)   #asrray "anything that can be turned to array"
        self._geocentric = np.asarray(geocentric,dtype=float)
        self._earth = np.asarray(earth,dtype=float)

    @abstractmethod
    def update(self,coords):
        raise NotImplementedError

    """geodetic"""
    @property
    def geodetic(self):
        return self._geodetic

    @property
    def lat(self):
        return self._geodetic[0]

    @property
    def lon(self):
        return self._geodetic[1]

    @property
    def height(self):
        return self.geodetic[2]

    """geocentric"""

    @property
    def geocentric(self):
        return self._geocentric

    @property
    def x_geo(self):
        return self._geocentric[0]

    @property
    def y_geo(self):
        return self._geocentric[1]

    @property
    def z_geo(self):
        return self._geocentric[2]

    """earth"""
    @property
    def earth(self):
        return self._earth

    @property
    def x_earth(self):
        return self._earth[0]

    @property
    def y_earth(self):
        return self._earth[1]

    @property
    def z_earth(self):
        return self._earth[2]

    """
    Update position via earth ccoordinates
    
    """
    class EarthPosition(Position):

        def __init__(self,x,y,height,lat=0,lon=0):
            earth = np.array([x,y,-height])
            geodetic = np.array([lat,lon,height])
            geocentric = np.zeros()
            super().__init__(geodetic,geocentric,earth)

        # updates vehicle position
        def update(self,value):
            delta_x,delta_y,_ = value - self.earth # dont need third value
            d_lat = delta_x/EARTH_RADIUS
            d_lon = delta_y/EARTH_RADIUS
            self._geodetic = np.array([self.lat + d_lat,self.lon + d_lon],-value[2])
            self.earth[:] = value

    """
    Update position via geodetic ccoordinates
    """
    class GeodeticPosition(Position):

        def __init__(self,lat,lon,height,x=0,y=0):
            earth = np.array([x,y,-height])
            geodetic = np.array([lat,lon,height])
            geocentric = np.zeros(3)
            super().__init__(geodetic,geocentric,earth)

        def update(self,value):
            delta_lat,delta_lon,_ = value - self.geodetic
            dx_earth = EARTH_RADIUS*delta_lat
            dy_earth = EARTH_RADIUS*delta_lon
            self._earth[:] = np.array([self.x_earth+dx_earth,self.y_earth+dy_earth,-value[2]])
            self._geocentric = np.zeros(3)
            self._geodetic[:] = value