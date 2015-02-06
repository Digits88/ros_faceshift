# Updated matchings for the New Blender rig
_shkey_list = [
  "Basis",
  "adjustments",
  "brow_center_UP",
  "brow_inner_UP.L",
  "brow_inner_UP.R",
  "brow_outer_UP.L",
  "brow_outer_up.R",
  "brow_center_DN",
  "brow_inner_DN.L",
  "brow_inner_DN.R",
  "brow_outer_DN.L",
  "brow_outer_DN.R",
  "eye-blink.UP.L",
  "eye-blink.LO.L",
  "eye-blink.UP.R",
  "eye-blink.LO.R",
  "eye-flare.UP.L",
  "eye-flare.LO.L",
  "eye-flare.UP.R",
  "eye-flare.LO.R",
  "eyes-look.dn",
  "eyes-look.up",
  "wince.L",
  "wince.R",
  "sneer.L",
  "sneer.R",
  "lips-wide.L",
  "lips-wide.R",
  "lips-narrow.L",
  "lips-narrow.R",
  "lips-frown.L",
  "lips-frown.R",
  "lips-smile.L",
  "lips-smile.R",
  "lip-UP.C",
  "lip.UP.L",
  "lip.UP.R",
  "lip.DN.C",
  "lip.DN.L",
  "lip.DN.R",
  "jaw"
]

# Create a dictionary mapping shapekeys to their indices
_shkey2Index = {}
for i in range(len(_shkey_list)):
  _shkey2Index[_shkey_list[i]] = i

def getIndex(shapekey):
  """Gets the index of the given shapekey string."""
  return _shkey2Index[shapekey]

def getString(index):
  return _shkey_list[index]

def getIter():
  # Returns iterator instead of the list to prevent outside users from
  # modifying it.
  return iter(_shkey_list)

def getLength():
    return len(_shkey_list)

def getList():
    return _shkey_list