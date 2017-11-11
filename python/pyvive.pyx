from libcpp.vector cimport vector
from libcpp.map cimport map
from libcpp.pair cimport pair
from libcpp.string cimport string
from libc.stdint cimport uint32_t

cdef extern from "vl_python.h":
    cdef cppclass ViveLibre:
        ViveLibre()
        map[uint32_t,vector[uint32_t]] poll_angles(string channel, uint32_t samples)
        pair[vector[float],vector[float]] poll_pnp(string channel, uint32_t samples)
        string get_config()

cdef class PyViveLibre:
    cdef ViveLibre* thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self):
        self.thisptr = new ViveLibre()
    def __dealloc__(self):
        del self.thisptr

    def poll_angles(self, string channel, uint32_t samples):
        return self.thisptr.poll_angles(channel, samples)

    def poll_pnp(self, string channel, uint32_t samples):
        return self.thisptr.poll_pnp(channel, samples)

    def get_config(self):
        return self.thisptr.get_config()
