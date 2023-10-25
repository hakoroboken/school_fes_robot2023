// Copyright 2023 Hakoroboken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

pub mod merger_module;

pub trait Invert {
    fn invert(&mut self);
}

impl Invert for bool {
    fn invert(&mut self) {
        *self = !*self;
    }
}

impl Invert for f32 {
    fn invert(&mut self) {
        *self *= -1.0;
    }
}