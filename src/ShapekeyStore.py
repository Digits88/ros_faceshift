#bit updated to match the FS2014.1
_shkey_list = [
  "00_EyeBlink_L",
  "01_EyeBlink_R",
  "02_EyeSquint_L",
  "03_EyeSquint_R",
  "04_EyeDown_L",
  "05_EyeDown_R",
  "08_EyeOpen_L",
  "09_EyeOpen_R",
  "12_EyeUp_L",
  "13_EyeUp_R",
  "14_BrowsD_I_L",
  "14_BrowsD_O_L",
  "15_BrowsD_O_R",
  "15_BrowsD_I_R",
  "16_BrowsU_C",
  "17_BrowsU_O_L",
  "17_BrowsU_I_L",
  "18_BrowsU_I_R",
  "18_BrowsU_O_R",
  "21_JawOpen",
  "26_MouthFrown_L",
  "27_MouthFrown_R",
  "28_MouthSmile_L",
  "29_MouthSmile_R",
  "32_LipsStretch_L",
  "33_LipsStretch_R",
  "361_LipsUpperUp_L",
  "362_LipsUpperUp_R",
  "371_LipsLowerDown_L",
  "372_LipsLowerDown_R",
  "38_LipsUpperOpen",
  "39_LipsLowerOpen",
  "40_LipsFunnel",
  "41_LipsPucker",
  "42_ChinLowerRaise",
  "43_ChinUpperRaise",
  "44_Sneer_C",
  "441_Sneer_L",
  "442_Sneer_R",
  "46_CheekSquint_L",
  "47_CheekSquint_R"
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