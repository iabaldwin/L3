import os
import h5py
import array
import numpy as np

class dataset:

    def __init__(self,path, limit='inf'):
    
        # Exists?
        if not os.path.exists( path ):
            raise Exception( "No such directory" )

        # Files exist?
        dataset_file = os.path.join( path, 'matlab', 'dataset' ) 
        if not os.path.exists( dataset_file ):
            raise Exception( "No matlab data" )
        
        self.dataset = h5py.File( dataset_file, 'r') 
        self.root    = self.dataset['dataset']

        # Trailing slash?
        if path[-1] == os.path.sep:
            path = path[:-1] 

        fqn = os.path.split(  path )

        self.dataset_path = path 
        self.dataset_name = fqn[1]

        self.target_directory = os.path.join( self.dataset_path, 'L3' )
        
        if not os.path.exists( self.target_directory ):
            os.mkdir( self.target_directory )

class Parser:

    def __init__(self, hdf_dataset_root ):
        self._dataset = hdf_dataset_root
        self._start = []
        self._limit = []
        self._binary = False

    def binary( self ):
        self._binary = True
        return self

    def write(self, directory):
      
        if not os.path.exists( directory ):
            raise BaseException()

        else:

            if self._binary:
                open_str = 'wb'
            else:
                open_str = 'w'

            with open( os.path.join( directory, self.name()  ), open_str ) as f:
                
                lower_limit = self._limit[0]
                upper_limit = self._limit[1]


                for entry in self._data:
       
                    if (lower_limit <=0 and upper_limit >= 0):
                        if self._binary:
                            f.write( entry )
                        else:
                            print >>f, entry

                    #self._limit[0] -= 1  
                    #self._limit[1] -= 1

                    lower_limit -= 1  
                    upper_limit -= 1

    # Helpers
    @staticmethod
    def hdf5ArrayToString( arr ):
        return ''.join( unichr(c) for c in arr )

class INS(Parser):

    def __init__(self, hdf_dataset_root, mission ):
        Parser.__init__(self, hdf_dataset_root)
    
        self.mission = mission

    def parse( self ):
        
        t = self._dataset['poses']['interpolated']['ins'][0]
        x = self._dataset['poses']['interpolated']['ins'][1]
        y = self._dataset['poses']['interpolated']['ins'][2]
        z = self._dataset['poses']['interpolated']['ins'][3]
        r = self._dataset['poses']['interpolated']['ins'][4]
        p = self._dataset['poses']['interpolated']['ins'][5]
        q = self._dataset['poses']['interpolated']['ins'][6]

        t = np.reshape( t, (1, t.size) )
        x = np.reshape( x, (1, t.size) )
        y = np.reshape( y, (1, t.size) )
        z = np.reshape( z, (1, t.size) )
        r = np.reshape( r, (1, t.size) )
        p = np.reshape( p, (1, t.size) )
        q = np.reshape( q, (1, t.size) )

        x -= self.mission.x
        y -= self.mission.y
        z -= self.mission.z

        #print self.mission.x
        #print self.mission.y
        #print self.mission.z

        #x -= 616648.701255
        #y -= 5742069.101970
        #z -= 73

        data = np.concatenate( (t, x, y, z, r, p, q), 0 ).transpose();

        if self._binary:
            self._data = map( lambda x: array.array( 'd', x.tolist()).tostring() , data)
        else:
            self._data = map( lambda x: ' '.join( map( lambda y: '%.18f' %y, x.tolist())), data)

        return self

    def duration( self, limit ):
        #self._limit = 100*limit     # Approx 100Hz
        #self._limit = (100*i for i in limit) # Approx 100Hz
        self._limit = (100*limit[0], 100*limit[1])
        return self

    def name(self):
        return 'OxTS.ins'

    def data( self ):
        return self._data

class LHLV(Parser):

    def __init__(self, hdf_dataset_root):
        Parser.__init__(self, hdf_dataset_root)
     
    def parse( self ):

        t = self._dataset['poses']['interpolated']['lhlv'][0]
        w1 = self._dataset['poses']['interpolated']['lhlv'][1]
        w2 = self._dataset['poses']['interpolated']['lhlv'][2]
        w3 = self._dataset['poses']['interpolated']['lhlv'][3]
        w4 = self._dataset['poses']['interpolated']['lhlv'][4]
        w5 = self._dataset['poses']['interpolated']['lhlv'][5]
        w6 = self._dataset['poses']['interpolated']['lhlv'][6]
        w7 = self._dataset['poses']['interpolated']['lhlv'][7]
        w8 = self._dataset['poses']['interpolated']['lhlv'][8]
        w9 = self._dataset['poses']['interpolated']['lhlv'][9]
        w10 = self._dataset['poses']['interpolated']['lhlv'][10]
        w11 = self._dataset['poses']['interpolated']['lhlv'][11]

        t = np.reshape( t, (1, t.size) )
        w1 = np.reshape( w1, (1, t.size) )
        w2 = np.reshape( w2, (1, t.size) )
        w3 = np.reshape( w3, (1, t.size) )
        w4 = np.reshape( w4, (1, t.size) )
        w5 = np.reshape( w5, (1, t.size) )
        w6 = np.reshape( w6, (1, t.size) )
        w7 = np.reshape( w7, (1, t.size) )
        w8 = np.reshape( w8, (1, t.size) )
        w9 = np.reshape( w9, (1, t.size) )
        w10 = np.reshape( w10, (1, t.size) )
        w11 = np.reshape( w11, (1, t.size) )

        data = np.concatenate( (t, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11 ), 0 ).transpose();
       
        if self._binary:
            self._data = map( lambda x: array.array( 'd', x.tolist()).tostring() , data)
        else:
            self._data = map( lambda x: ' '.join( map( lambda y: '%.18f' %y, x.tolist())), data)

        return self

    def name(self):
        return 'OxTS.lhlv'

    def duration( self, limit ):
        #self._limit = 100*limit     # Approx 100Hz
        #self._limit = (100*i for i in limit) 
        self._limit = (100*limit[0], 100*limit[1])# Approx 100Hz
        return self


class LIDAR(Parser):

    def __init__(self, hdf_dataset_root):
        Parser.__init__(self, hdf_dataset_root)
        self._data = []

    def parse( self ):

        # LIDARs in the dataset
        for lidar in self._dataset['lasers']:

            # LIDAR structures
            for iterator in lidar:
                
                data = self._dataset[iterator]

                LIDAR_name = Parser.hdf5ArrayToString( data['name'].value )

                # Add in here, reflectances
                self._data.append( (LIDAR_name,data['ranges'].value.transpose()) )

        return self

    def duration( self, limit ):
        #self._limit = 50*limit     # Approx 50Hz
        self._limit = (50*limit[0], 50*limit[1])
        return self

    def write( self, directory ):

        if not os.path.exists( directory ):
            raise BaseException()

        else:

            for lidar in self._data:

                lower_limit = self._limit[0]
                upper_limit = self._limit[1]


                with open( os.path.join( directory, lidar[0] + '.lidar', ), 'wb' ) as f:
                    
                    for entry in lidar[1]:

                        if (lower_limit <=0 and upper_limit >= 0 ):
                        
                            if self._binary:
                                arr =  array.array('d', entry.tolist() )
                                f.write( arr.tostring())
                            
                            else:
                                data = map( lambda x: '%.18f' % x, entry.tolist() ) 
                                print >>f, ' '.join( data )
                        

                        lower_limit -= 1  
                        upper_limit -= 1
