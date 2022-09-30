#!/bin/bash
glslangValidator -V render_unfolded.frag -o render_unfolded_frag.spv
glslangValidator -V render_unfolded.vert -o render_unfolded_vert.spv
xxd -i render_unfolded_vert.spv > render_unfolded_vert.h
xxd -i render_unfolded_frag.spv > render_unfolded_frag.h
