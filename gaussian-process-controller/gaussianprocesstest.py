import gp_controller as gpc
import iCubInterface
import numpy as np

def getState(iCubI):
  tactileData = iCubI.readTactileData()
  encodersData = iCubI.readEncodersData()
  return [tactileData,encodersData]

def main():
  
	gp = gpc.GPController('2_2')
	gp.load_controller()

	iCubI = iCubInterface.ICubInterface('iCubInterface')
	iCubI.loadInterfaces()
	 
	for i in range(10)
	
		
		state = getState(iCubI)
		tactileData = state[0]
		encodersData = state[1]
		
		
		
		
		
	for i in range(10):
		ft = np.asarray([3*i,0.21628,0.98986,1.4574,0.51051,0.6026] )
		print 'input\t', ft
		musigma = gp.get_mu_sigma(ft)[0]
		print 'mu\t', musigma[0]
		print 'sigma\t', musigma[1]
		print ''

	dat = np.asarray([[ 0,0.21628,0.98986,1.4574,0.51051,0.6026],\
			   [0.12343,0.32001,1.5912,1.1571,0.62898,0.38699],\
			   [0.17078,0.83327,1.8773,1.0516,0.47641,0.31159],\
			   [0.19378,0.28035,1.8988,1.0186,0.45857,0.29359],\
			   [0.19378,0.35713,1.9675,0.94231,0.47732,0.25338]])

	datstdev = np.std(dat,0)
	datmean = np.mean(dat,0)

	r = np.random.randn(10,6)
	res = r * datstdev + datmean

	for i in range(10):
		print 'input\t', res[i]
		musigma = gp.get_mu_sigma(res[i])[0]
		print 'mu\t', musigma[0]
		print 'sigma\t', musigma[1]
		print ''

		
if __name__ == "__main__":
    main()