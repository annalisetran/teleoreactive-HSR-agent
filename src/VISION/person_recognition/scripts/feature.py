#!/usr/bin/env python3
class Feature:
    def __init__(self, perspective, feature_type, encoding, colour=None):
        self.perspective = perspective
        self.feature_type = feature_type
        self.encoding = encoding
        self.colour = colour

    def __str__(self):
        return f"perspective: {self.perspective}; feature_type: {self.feature_type}; encoding: {self.encoding} colour: {self.colour}"
    

    def update(self, perspective, feature_type, encoding, colour=None):
        self.perspective = perspective
        self.feature_type = feature_type
        self.encoding = encoding 
        self.colour = colour     