# ros_faceshift

Listens for default faceshift port 33433. UDP protocol should be selected for streaming on faceshift.
Decode network messages to PAU message and publishes it.
  * Works only with 2014.1 version and expects default tracking features (total 51) to be streamed.
  * See 6.6.2 Chapter fo network potocol for more information [http://doc.faceshift.com/studio/faceshift-doc-complete-htmlch6.html]


### published topics
  * /faceshift_track - publishes `pau2motors/pau` messages based on faceshift input. 

