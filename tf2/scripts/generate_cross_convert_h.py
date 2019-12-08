# make string literals unicode default in Python 2
from __future__ import unicode_literals
from sys import argv


class Namespace(object):
    def __init__(self, name):
        self._name = name
        self._members = []

    def write(self, out_file):
        if self._name == "":
            # global namespace
            self._write_members(out_file)
            out_file.write("\n\n")
        else:
            out_file.write("namespace " + self._name + " {\n")
            self._write_members(out_file)
            out_file.write("}\n\n")

    def _write_members(self, out_file):
        for i in self._members:
            i.write(out_file)

    def add_member(self, a_member):
        self._members.append(a_member)
        return self

    def write_name(self, out_file):
        out_file.write(self._name + "::")

    def write_includes(self, out_file):
        includes = set()
        for i in self._members:
            includes = i.add_include_header_to_set(includes)
        out_file.write("\n".join(includes))
        out_file.write("\n\n")



class Template(object):
    def __init__(self, parameters):
        self._parameters = parameters
    def write_declaration(self, out_file):
        params = ["%s %s" % i for i in self._parameters]
        if len(params) == 0:
            return
        out_file.write("template<" + ", ".join(params) + ">\n")
    def write_params_for_specialisation(self, out_file, param_range=slice(None)):
        params = [i[1] for i in self._parameters[param_range]]
        if len(params) == 0:
            return
        out_file.write("<" + ",".join(params) + ">")
    def get_types(self):
        return tuple(i[0] for i in self._parameters)

class CxxClassBase(object):
    def __init__(self, name, namespace=None, stamped=False):
        self._name = name
        if namespace is None:
            self._namespace = None
        else:
            self._namespace = namespace.add_member(self)
        self._stamped = stamped
    def add_include_header_to_set(self, includes):
        return includes

    def stamped(self):
        return self._stamped

    def compare_stamped(self, other):
        if self.stamped() == other.stamped() == "both":
            return "both"
        elif self.stamped() and other.stamped():
            return True
        else:
            return False


class CxxClass(CxxClassBase):
    def __init__(self, name, namespace=None, template=None, stamped=False):
        super(CxxClass, self).__init__(name, namespace, stamped)
        self._template = template
    def write(self, out_file):
        """write forward declaration"""
        if self._template is not None:
            self._template.write_declaration(out_file)
        out_file.write("class " + self._name + ";\n")
    def get_template_types(self):
        if self._template is None:
            return tuple()
        else:
            return self._template.get_types()
    def write_name(self, out_file):
        if self._namespace is None:
            out_file.write("::")
        else:
            self._namespace.write_name(out_file)
        out_file.write(self._name)

class TypeMap(CxxClassBase):
    def __init__(self, msgs_name, class_a, class_b, namespace, stamped = False):
        super(TypeMap, self).__init__("BidirectionalTypeMap", namespace, stamped)
        self._msgs_name = msgs_name
        if stamped:
            self._msgs_name += "Stamped"
        self._classes = (class_a, class_b)
        # build template arguments if necessary
        self._templates_count = [0, 0]
        template_types = []
        for i, c in enumerate(self._classes):
            needed_types = list(c.get_template_types())
            template_types += needed_types
            self._templates_count[i] = len(needed_types)
        if len(template_types) == 0:
            self._template = None
        else:
            self._template = Template([(t, "T%d" %i) for i, t in enumerate(template_types)])

    def write(self, out_file):
        if self._template:
            self._template.write_declaration(out_file)
        else:
            out_file.write("template<>\n")
        out_file.write("struct %s<" % self._name)

        self._write_class_name(out_file, 0)
        out_file.write(", ")
        self._write_class_name(out_file, 1)

        out_file.write("> {\nusing type = ::geometry_msgs::" + self._msgs_name + ";\n};\n\n")

    def _write_class_name(self, out_file, idx):
        if self.stamped():
            out_file.write("::tf2::Stamped<")
        self._classes[idx].write_name(out_file)
        if self._template:
            if idx == 0:
                s = slice(0, self._templates_count[0])
            else:
                s = slice(self._templates_count[0], None)
            self._template.write_params_for_specialisation(out_file, s)
        if self.stamped():
            out_file.write(">")

    def add_include_header_to_set(self, includes):
        includes.add("#include <geometry_msgs/%s.h>" % self._msgs_name)
        return includes

def TypeMapIterator(msgs_name, namespace, classes):
    """
Iterate pairwise over classes and create TypeMap for each pair.
For example, for classes=[a, b, c, d] it yields [ Typemap(a,b), Typemap(a,c),
Typemap(a,d), Typemap(b,c), Typemap(b,d), Typemap(c,d)].
The stamped attribute of each class is honored.
"""
    for i in range(0, len(classes)):
        for next_class in classes[i+1 : ]:
            is_stamped = next_class.compare_stamped(classes[i])
            if is_stamped: # True or "both"
                yield TypeMap(msgs_name, classes[i], next_class, namespace, True)
            if (not is_stamped) or (is_stamped == "both"): # False or "both"
                yield TypeMap(msgs_name, classes[i], next_class, namespace, False)


def build_namespaces():
    # global namespace
    ns_global = Namespace("")

    btVector3 = CxxClass("btVector3", ns_global, stamped=True)

    # Eigen
    ns_eigen = Namespace("Eigen")
    eigen_vec3 =  CxxClass("Matrix", ns_eigen, Template([("typename", "T"), ("int", "_rows"), ("int", "_cols"), ("int", "_options"), ("int", "_maxrows"), ("int", "maxcols")]), stamped="both")
    eigen_quat = CxxClass("Quaternion", ns_eigen, Template([("typename", "T"), ("int", "_options")]), stamped="both")
    # Affine and Isometry
    eigen_transform = CxxClass("Transform", ns_eigen, Template([("typename", "T"), ("int", "_dim"), ("int", "_mode"), ("int", "_options")]), stamped="both")

    # KDL
    ns_kdl = Namespace("KDL")
    kdl_frame = CxxClass("Frame", ns_kdl, stamped="both")
    kdl_vec3 = CxxClass("Vector", ns_kdl, stamped=True)
    kdl_twist = CxxClass("Twist", ns_kdl, stamped=True)
    kdl_wrench = CxxClass("Wrench", ns_kdl, stamped=True)

    # tf2
    ns_tf2_forward = Namespace("tf2")
    tf2_vec3 = CxxClass("Vector3", ns_tf2_forward, stamped="both")
    tf2_quat = CxxClass("Quaternion", ns_tf2_forward, stamped="both")
    tf2_transform = CxxClass("Transform", ns_tf2_forward, stamped="both")
    tf2_wrench = CxxClass("Wrench", ns_tf2_forward, stamped=True)

    maps = {
        "Pose":       [tf2_transform, eigen_transform, kdl_frame],
        #"Vector3":    [eigen_vec3, tf2_vec3],
        "Point":      [eigen_vec3, tf2_vec3, kdl_vec3, btVector3],
        "Quaternion": [eigen_quat, tf2_quat],
        "Wrench":     [tf2_wrench, kdl_wrench],
    }

    ns_tf2 = Namespace("tf2")

    for msgs_name, classes in maps.items():
        for mapping in TypeMapIterator(msgs_name, ns_tf2, classes):
            pass

    return (ns_global, ns_eigen, ns_kdl, ns_tf2_forward, ns_tf2)

hdr_begin = """/*
 * Copyright (c) 2013, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


 // NOTE: This file is generated automatically, do not edit.

#ifndef TF2_IMPL_CROSS_CONVERT_H
#define TF2_IMPL_CROSS_CONVERT_H

#include <tf2/transform_functions.h>
#include <tf2/transform_datatypes.h>

"""

if __name__ == "__main__":
    with open(argv[1],"w") as f:
        f.write(hdr_begin)
        namespaces = build_namespaces()
        namespaces[-1].write_includes(f)

        for ns in namespaces:
            ns.write(f)

        f.write("#endif // TF2_IMPL_CROSS_CONVERT_H\n")
