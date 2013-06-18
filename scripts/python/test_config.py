import os

config = Config ()

config.readFile ( '/Users/ian/code/datasets/configuration/missions/2011-04-18-15-38-063dBumbRTK2DLaser.config' )

print config.value( 'mission.description' )


#self.assert_ ( config.value ( 'test' )[0] == 'value' )
#self.assert_ ( config.value ( 'test' )[1] == True )


