
----------------------------------
-- dependencies
----------------------------------
require 'torch'
require 'image'
require 'libkinect'

----------------------------------
-- package
----------------------------------
kinect = {}

-- Initializes the Kinect.
-- 
-- Args:
--   kinectParams - a table of kinect calibration parameters. These will,
--                  generally speaking, be different for each Kinect device.
function kinect.init(kinectParams)
   if not kinect.initialized then
	 
      kinect.rgb = torch.Tensor()
      kinect.depth = torch.Tensor()
			kinect.projDepth = torch.Tensor()
			kinect.filtered = torch.Tensor()
			
			kinect.params = kinectParams
			
      torch.Tensor().kinect.init()
      kinect.initialized = true
   end
end

function kinect.stop()
  local type = kinect.rgb
	type.kinect.stop()
	kinect.initialized = false
end

function kinect.getRGBD()
   local type = kinect.rgb
   type.kinect.getRGBD(kinect.rgb, kinect.depth)
   return kinect.rgb, kinect.depth
end

function kinect.project(depth)
  local type = kinect.rgb
	type.kinect.project(depth, kinect.projDepth)
	return kinect.projDepth
end

function kinect.projectPar(depth)
  local type = kinect.rgb
	type.kinect.project_par(depth, kinect.projDepth)
	return kinect.projDepth
end

function kinect.filterCbf(depthProj, rgb)
  local type = kinect.rgb
	type.kinect.filterCbf(depthProj, rgb, kinect.filtered)
	return kinect.filtered
end

----------------------------------
-- package a little demo
----------------------------------
kinect.testme = function()
                   print '--------------------------------------------------'
                   print 'grabbing frames from kinect'
                   kinect.init()
                   local fps = 0
                   for i = 1,200 do -- ~10 seconds
                      sys.tic()
                      local frame, depth = kinect.getRGBD()
                      w = image.display{image=frame, win=w, legend='rgb ['..fps..'fps]'}
                      wd = image.display{image=depth, win=wd, legend='depth ['..fps..'fps]'}
                      local t = sys.toc()
                      if fps == 0 then fps = 1/t end
                      fps = math.ceil((1/t + fps)/2)
                   end
                   print '--------------------------------------------------'
                end
