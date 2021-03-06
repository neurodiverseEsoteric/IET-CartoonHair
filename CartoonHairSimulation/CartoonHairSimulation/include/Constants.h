#include "stdafx.h"
/*
This file contains a number of pre-processor directives that control the initial values of many of the hair
parameters. For example: the stiffness of of the edge, torsion or bending springs.

The stylised specular option has become somewhat non-functional in this version as it was not actively developed after it was
found to be unsuitable for 3D hair strands.
*/

//CUSTOM BUILD OPTIONS
#define EDGE_STIFFNESS 1.0f
#define TORSION_STIFFNESS 0.01f
#define BENDING_STIFFNESS 0.01f
#define ANCHOR_STIFFNESS 0.01f
#define ANIMATION_SPEED 0.4
#define HAIR_QUADRATIC_A -13.9
#define HAIR_QUADRATIC_B 4.9
#define HAIR_QUADRATIC_C 6.4


//SILHOUETTE OPTIONS
#define OUTER_SILHOUETTE
#define X_DILATION 3
#define Y_DILATION 3
#define EDGE_THRESHOLD 2.0f
#define SILHOUETTE_A 0.0f
#define SILHOUETTE_B 0.0f
#define SILHOUETTE_C 1.0f


//TYPES OF SPRINGS IN THIS BUILD
#define EDGE_SPRINGS
#define BENDING_SPRINGS
#define TORSION_SPRINGS
#define ANCHOR_SPRINGS
#define GHOST_STRAND


//TYPES OF STROKE QUAD SCALING
#define STROKE_SCALE 0.02f
#define STROKE_LIMIT 0.3f


//ANCHOR BLENDING OPTIONS
#define BLENDING_QUADRATIC_A 0.0f
#define BLENDING_QUADRATIC_B 0.0f
#define BLENDING_QUADRATIC_C 1.0f


//HATCHING OPTIONS
#define HATCHING_STROKE_SCALE 1.0f


//ID BUFFER COLOUR INCREMENT BETWEEN NEW IDs
#define ID_INCREMENT 0.05f


//ANCHOR ANIMATION SPEED SCALE
#define ANCHOR_FRAME_INCRMENT 0.01f


//HAIR STRAND SETTINGS
#define NUM_HAIR_SAMPLES 8
#define NUM_HAIR_SHAPE_SAMPLES 8


//BULLET PHYSICS collision groups used to limit collisions between certain groups
#define BODY_GROUP 0x1
#define HAIR_GROUP 0x2
#define GHOST_GROUP 0x4
#define MIN_MARGIN 0.25f
#define MAX_MARGIN 0.25f


//based on http://bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
#define FIXED_TIMESTEP btScalar(1.0f)/btScalar(60.0f);
#define MAX_SUB_STEPS 1


//SPECULAR HIGHLIGHTS settings
//#define STYLISED_SPECULAR
#define SPECULAR_WEIGHT 0.3f
#define SHININESS 1.0f
#define KS 1.0f
#define SPECULAR_THRESHOLD 10.0f
#define MAX_MERGING_DISTANCE 2.0f
#define MIN_MERGING_DISTANCE 1.0f
#define MAX_GROUP_SECTIONS 100
#define TIP_EXCLUDE 0.4
#define HIGHLIGHT_SCALE 0.04f
#define MAX_GROUP_SIZE 5
#define MIN_GROUP_SIZE 2
#define ZMIN 10.0f
#define R 4.0f
#define BLINN_S 1.0f
#define SPECULAR_TEXTURE_S 1.0f
#define BACKLIGHTING_TEXTURE_S 1.0f


//VECTOR RESERVE SIZES
#define CANDIDATES_RESERVE_SIZE 100
#define TEMP_SILHOUETTE_RESERVE_SIZE 40


//HAIR COLOUR
#define RED 1.0f
#define GREEN 0.0f
#define BLUE 0.0f

