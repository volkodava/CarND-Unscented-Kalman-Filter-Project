#!/bin/bash

mkdir `echo $1 | tr '[:upper:]' '[:lower:]' | tr " " "_" | tr "." "_"`
