import numpy as np, cv2, imageio
import os
import depth_utils as du
import rotation_utils as ru
from fmm_planner import FMMPlanner
import skimage
import matplotlib.pyplot as plt
import subprocess as sp

def subplot(plt, Y_X, sz_y_sz_x = (10, 10)):
    Y,X = Y_X
    sz_y, sz_x = sz_y_sz_x
    plt.rcParams['figure.figsize'] = (X*sz_x, Y*sz_y)
    fig, axes = plt.subplots(Y, X)
    plt.subplots_adjust(wspace=0.1, hspace=0.1)
    return fig, axes

class DepthMapperAndPlanner(object):
  def __init__(self, dt=10, camera_height=125., upper_lim=150., map_size_cm=6000, out_dir=None,
      mark_locs=False, reset_if_drift=False, count=-1, close_small_openings=False,
      recover_on_collision=False, fix_thrashing=False, goal_f=1.1, point_cnt=2):
    self.map_size_cm = map_size_cm
    self.dt = dt
    self.count = count
    self.out_dir = out_dir
    self.mark_locs = mark_locs
    self.reset_if_drift = reset_if_drift
    self.elevation = 0. #np.rad2deg(env_config.SIMULATOR.DEPTH_SENSOR.ORIENTATION[0])
    self.camera_height = camera_height
    self.upper_lim = upper_lim
    self.lower_lim = 20
    self.close_small_openings = close_small_openings
    self.num_erosions = 2
    self.recover_on_collision = recover_on_collision
    self.fix_thrashing = fix_thrashing
    self.goal_f = goal_f
    self.point_cnt = point_cnt
    print(self.elevation, self.camera_height, self.upper_lim, self.lower_lim)

  def reset(self):
    self.RESET = True
     
  def _reset(self, goal_dist, soft=False):
    # Create an empty map of some size
    resolution = self.resolution = 5
    self.selem = skimage.morphology.disk(10 / resolution)
    self.selem_small = skimage.morphology.disk(1)
    # 0 agent moves forward. Agent moves in the direction of +x
    # 1 rotates left 
    # 2 rotates right
    # 3 agent stop
    self.z_bins = [self.lower_lim, self.upper_lim]
    map_size_cm = np.maximum(self.map_size_cm, goal_dist*2*self.goal_f) // resolution
    map_size_cm = int(map_size_cm * resolution)
    self.map = np.zeros((map_size_cm//resolution+1, map_size_cm//resolution+1, len(self.z_bins)+1), dtype=np.float32)
    self.loc_on_map = np.zeros((map_size_cm//resolution+1, map_size_cm//resolution+1), dtype=np.float32)
    self.current_loc = np.array([(self.map.shape[0]-1)/2, (self.map.shape[0]-1)/2, 0], np.float32)
    self.current_loc[:2] = self.current_loc[:2]*resolution
    self.camera = du.get_camera_matrix(256, 256, 90)
    self.goal_loc = None
    self.last_act = 3
    self.locs = []
    self.acts = []
    self.last_pointgoal = None
    if not soft:
      self.num_resets = 0
      self.count = self.count+1
      self.trials = 0
      self.rgbs = []
      self.depths = []
      self.recovery_actions = []
      self.thrashing_actions = []
  
  def add_observation(self, depth):
    # depth is in cm
    d = depth[:,:,0]*1
    d[d == 0] = np.NaN
    d[d > 990] = np.NaN
    XYZ1 = du.get_point_cloud_from_z(d, self.camera);
    # print(np.min(XYZ1.reshape(-1,3), 0), np.max(XYZ1.reshape(-1,3), 0))
    XYZ2 = du.make_geocentric(XYZ1*1, self.camera_height, self.elevation)
    # print(np.min(XYZ2.reshape(-1,3), 0), np.max(XYZ2.reshape(-1,3), 0))
    # Transform pose
    # Rotate and then translate by agent center
    XYZ3 = self.transform_to_current_frame(XYZ2)
    # print(np.min(XYZ3.reshape(-1,3), 0), np.max(XYZ3.reshape(-1,3), 0))
    counts, is_valids = du.bin_points(XYZ3, self.map.shape[0], self.z_bins, self.resolution)
    self.map = self.map + counts
  
  def plan_path(self, goal_loc):
    state = self.current_loc*1.
    state[:2] = state[:2]/self.resolution
    
    obstacle = self.map[:,:,1] >= self.point_cnt
    traversible = skimage.morphology.binary_dilation(obstacle, self.selem) != True
    if self.mark_locs:
      traversible_locs = skimage.morphology.binary_dilation(self.loc_on_map, self.selem) == True
      traversible = np.logical_or(traversible_locs, traversible)
    
    if self.close_small_openings:
      n = self.num_erosions
      reachable = False
      while n >= 0 and not reachable:
        traversible_open = traversible.copy()
        for i in range(n):
          traversible_open = skimage.morphology.binary_erosion(traversible_open, self.selem_small)
        for i in range(n):
          traversible_open = skimage.morphology.binary_dilation(traversible_open, self.selem_small)
        planner = FMMPlanner(traversible_open, 360//self.dt)
        goal_loc_int = goal_loc // self.resolution
        goal_loc_int = goal_loc_int.astype(np.int32)
        reachable = planner.set_goal(goal_loc_int)
        reachable = reachable[int(round(state[1])), int(round(state[0]))]
        n = n-1
    else:
      planner = FMMPlanner(traversible, 360//self.dt)
      goal_loc_int = goal_loc // self.resolution
      goal_loc_int = goal_loc_int.astype(np.int32)
      reachable = planner.set_goal(goal_loc_int)
    self.fmm_dist = planner.fmm_dist*1
    a, state, act_seq = planner.get_action(state)
    for i in range(len(act_seq)):
      if act_seq[i] == 3:
        act_seq[i] = 0
      elif act_seq[i] == 0:
        act_seq[i] = 3
    if a == 3:
      a = 0
    elif a == 0:
      a = 3
    return a, act_seq

  def get_best_action(self):
    None

  def transform_to_current_frame(self, XYZ):
      R = ru.get_r_matrix([0.,0.,1.], angle=self.current_loc[2]-np.pi/2.)
      XYZ = np.matmul(XYZ.reshape(-1,3), R.T).reshape(XYZ.shape)
      XYZ[:,:,0] = XYZ[:,:,0] + self.current_loc[0]
      XYZ[:,:,1] = XYZ[:,:,1] + self.current_loc[1]
      return XYZ

  def update_loc(self, last_act, pointgoal=None):
    # Currently ignores goal_loc.
    if last_act == 1:
      self.current_loc[2] = self.current_loc[2] + self.dt*np.pi/180.
    elif last_act == 2:
      self.current_loc[2] = self.current_loc[2] - self.dt*np.pi/180.
    elif last_act == 0:
      self.current_loc[0] = self.current_loc[0] + 25*np.cos(self.current_loc[2])
      self.current_loc[1] = self.current_loc[1] + 25*np.sin(self.current_loc[2])
    self.locs.append(self.current_loc+0)
    self.mark_on_map(self.current_loc)
  
  def mark_on_map(self, loc):
    x = int(loc[0] // self.resolution) 
    y = int(loc[1] // self.resolution) 
    self.loc_on_map[y,x] = 1
  
  def save_vis(self):
    if self.trials < 20:
      fig, axes = subplot(plt, (1,3))
      axes = axes.ravel()[::-1].tolist()
      ax = axes.pop()

      locs = np.array(self.locs).reshape([-1,3])
      acts = np.array(self.acts).reshape([-1])
      ax.imshow(self.map[:,:,1] > 0, origin='lower')
      ax.plot(locs[:,0]/5, locs[:,1]/5, 'm.', ms=3)
      if locs.shape[0] > 0:
        ax.plot(locs[0,0]/5, locs[0,1]/5, 'bx')
      ax.plot(self.current_loc[0]/5, self.current_loc[1]/5, 'b.')
      ax.plot(self.goal_loc[0]/5, self.goal_loc[1]/5, 'y*')

      ax = axes.pop()
      ax.imshow(self.fmm_dist, origin='lower')
      ax.plot(locs[:,0]/5, locs[:,1]/5, 'm.', ms=3)
      if locs.shape[0] > 0:
        ax.plot(locs[0,0]/5, locs[0,1]/5, 'bx')
      ax.plot(self.current_loc[0]/5, self.current_loc[1]/5, 'b.')
      ax.plot(self.goal_loc[0]/5, self.goal_loc[1]/5, 'y*')

      ax = axes.pop()
      ax.plot(acts)
      plt.savefig(os.path.join(
        self.out_dir, '{:04d}_{:03d}.png'.format(self.count, self.trials)),
        bbox_inches='tight')
      plt.savefig(os.path.join(self.out_dir, '{:04d}.png'.format(self.count)),
        bbox_inches='tight')
      plt.close()
  
  def soft_reset(self, pointgoal):
    # This reset is called if there is drift in the position of the goal
    # location, indicating that there had been collisions.
    if self.out_dir is not None:
      self.save_vis()
    self._reset(pointgoal[0]*100., soft=True)
    self.trials = self.trials+1
    self.num_resets = self.num_resets+1
    xy = self.compute_xy_from_pointnav(pointgoal)
    # self.current_loc has been set inside reset
    self.goal_loc = xy*1
    self.goal_loc[0] = self.goal_loc[0] + self.current_loc[0]
    self.goal_loc[1] = self.goal_loc[1] + self.current_loc[1]
    self.mark_on_map(self.goal_loc)
    self.mark_on_map(self.current_loc)
    if self.num_resets == 6:
      # We don't want to keep resetting. First few resets fix themselves,
      # so do it for later resets.
      num_rots = int(np.round(180 / self.dt))
      self.recovery_actions = [1]*num_rots + [0]*6
    else:
      self.recovery_actions = []
  
  def check_drift(self, pointgoal):
    xy = self.compute_xy_from_pointnav(pointgoal)
    goal_loc = xy*1
    goal_loc[0] = goal_loc[0] + self.current_loc[0]
    goal_loc[1] = goal_loc[1] + self.current_loc[1]
    # np.set_printoptions(precision=3, suppress=True)
    # print(self.last_act, self.current_loc, goal_loc, self.goal_loc, xy, pointgoal)
    return np.linalg.norm(goal_loc - self.goal_loc) > 5

  def check_thrashing(self, n, acts):
    thrashing = False
    if len(acts) > n:
      last_act = acts[-1]
      thrashing = last_act == 1 or last_act == 2
      for i in range(2, n+1):
        if thrashing:
          thrashing = acts[-i] == 3-last_act
          last_act = acts[-i]
        else:
          break
    return thrashing
  
  def compute_xy_from_pointnav(self, pointgoal):
    xy = np.array([np.cos(pointgoal[1]+self.current_loc[2]), 
                   np.sin(pointgoal[1]+self.current_loc[2])], dtype=np.float32)
    xy = xy*pointgoal[0]*100
    return xy
    
  def act(self, observations):
    if self.RESET:
      self.RESET = False
      return self._act(0, observations, True)
    else:
      return self._act(0, observations, False)
  
  def _act(self, i, obs, done):
    # depth = obs['depth'][i,...].detach().cpu().numpy()
    # pointgoal = obs['pointgoal'][i,...].detach().cpu().numpy()
    # rgb = obs['rgb'][i,...].detach().cpu().numpy().astype(np.uint8)
    rgb = obs['rgb'].astype(np.uint8)
    depth = obs['depth']
    pointgoal = obs['pointgoal']
    # print(np.min(depth), np.max(depth))
    if done:
      if self.out_dir is not None and hasattr(self, 'locs'):
        self.save_vis()
        if self.last_pointgoal is not None and self.last_pointgoal[0] > 1:
          self.write_mp4_imageio()
      self._reset(pointgoal[0]*100.)
      # self.current_loc has been set inside reset
      xy = self.compute_xy_from_pointnav(pointgoal)
      self.goal_loc = xy*1
      self.goal_loc[0] = self.goal_loc[0] + self.current_loc[0]
      self.goal_loc[1] = self.goal_loc[1] + self.current_loc[1]
      self.mark_on_map(self.goal_loc)
      self.mark_on_map(self.current_loc)
    
    self.update_loc(self.last_act)
    drift = self.check_drift(pointgoal)
    if self.reset_if_drift and drift:
      # import pdb; pdb.set_trace()
      self.soft_reset(pointgoal)


    self.add_observation(depth*1000)
    act, act_seq = self.plan_path(self.goal_loc)
    
    if self.recover_on_collision:
      if len(self.recovery_actions) > 0:
        act = self.recovery_actions[0] 
        self.recovery_actions = self.recovery_actions[1:]
    
    thrashing = self.check_thrashing(8, self.acts)
    if thrashing and len(self.thrashing_actions) == 0:
      self.thrashing_actions = act_seq 
      # print(1, self.thrashing_actions)
    
    if self.fix_thrashing:
      if len(self.thrashing_actions) > 0:
        act = self.thrashing_actions[0] 
        self.thrashing_actions = self.thrashing_actions[1:]
        # print(2, self.thrashing_actions)

    self.acts.append(act)
    self.last_act = act 
    
    self.depths.append((depth[...,0]*255).astype(np.uint8))
    self.rgbs.append(rgb)
    self.last_pointgoal = pointgoal + 0
    
    return act
  
  def write_mp4_imageio(self):
    sz = self.rgbs[0].shape[0]
    out_file_name = os.path.join(self.out_dir, '{:04d}.gif'.format(self.count))
    imageio.mimsave(out_file_name, self.rgbs)
    
    sz = self.depths[0].shape[0]
    out_file_name = os.path.join(self.out_dir, '{:04d}_d.gif'.format(self.count))
    imageio.mimsave(out_file_name, self.depths)

  def write_mp4_cv2(self):
    sz = self.rgbs[0].shape[0]
    out_file_name = os.path.join(self.out_dir, '{:04d}.mp4'.format(self.count))
    video = cv2.VideoWriter(out_file_name, -1, 10, (sz, sz))
    for rgb in self.rgbs:
      video.write(rgb[:,:,::-1])
    video.release()

  def write_mp4(self):
    sz = self.depths[0].shape[0]
    out_file_name = os.path.join(self.out_dir, '{:04d}.mp4'.format(self.count))
    ffmpeg_bin = 'ffmpeg'
    command = [ffmpeg_bin,
        '-y', # (optional) overwrite output file if it exists
        '-f', 'rawvideo',
        '-vcodec','rawvideo',
        '-s', '{:d}x{:d}'.format(sz,sz), # size of one frame
        '-pix_fmt', 'rgb24',
        '-r', '4', # frames per second
        '-i', '-', # The imput comes from a pipe
        '-an', # Tells FFMPEG not to expect any audio
        '-vcodec', 'mpeg', out_file_name]
    pipe = sp.Popen(command, stdin=sp.PIPE, stderr=sp.PIPE)
    for rgb in self.rgbs:
      pipe.proc.stdin.write(rgb.tostring())
    # self.add_observation(depth, goal_vec, self.last_act)
    # act = get_action(self.map)
    # self.last_act = act
