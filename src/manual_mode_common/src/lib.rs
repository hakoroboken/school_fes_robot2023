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

pub trait Converter {
    fn make_f32(&self) -> f32;
}

impl Converter for remote_control_msgs::msg::Dpad {
    fn make_f32(&self) -> f32{
        if self.up && !self.down {
            return 1.0;
        }else if !self.up && self.down{
            return -1.0;
        }else{
            return 0.0;
        }
    }
}


pub trait GetGamePadValue {
    fn get_button(&self, name: &str)->bool;
    fn get_value(&self, name: &str) -> f32;
}

impl GetGamePadValue for remote_control_msgs::msg::Gamepad {
    fn get_button(&self, name: &str)->bool{
        match  name{
            "x" => self.button.x,
            "y" => self.button.y,
            "a" => self.button.a,
            "b" => self.button.b,
            "up" => self.dpad.up,
            "down" => self.dpad.down,
            "left" => self.dpad.left,
            "right" => self.dpad.right,
            "trigger.left" => self.left_trigger.button,
            "trigger.right" => self.right_trigger.button,
            "joystic.left" => self.left_joystic.thumbstick_button,
            "joystic.right" => self.right_joystic.thumbstick_button,
            "shoulder.left" => self.left_shoulder_button,
            "shoulder.right" => self.right_shoulder_button,
            _ => false,
        }
    }
    fn get_value(&self, name: &str) -> f32{
        match  name{
            "dpad" => self.dpad.make_f32(),
            "x.joystic.left" => self.left_joystic.x,
            "y.joystic.left" => self.left_joystic.y,
            "x.joystic.right" => self.right_joystic.x,
            "y.joystic.right" => self.right_joystic.y,
            "trigger.left" => self.left_trigger.value,
            "trigger.right" => self.right_trigger.value,
            _ => 0.0
        }
    }
}



pub fn check_pram_button_name(name: &str){
    if "x" != name &&
        "y" != name &&
        "a" != name &&
        "b" != name &&
        "up" != name &&
        "down" != name &&
        "left" != name &&
        "right" != name &&
        "trigger.left" != name &&
        "trigger.right" != name &&
        "joystic.right" != name &&
        "joystic.left" != name &&
        "shoulder.left" != name &&
        "shoulder.right" != name
    {
        panic!("Parameter ERROR")
    }
}

pub fn check_pram_value_name(name: &str){
    if  "dpad" != name &&
        "trigger.left" != name &&
        "trigger.right" != name &&
        "x.joystic.right" != name &&
        "y.joystic.right" != name &&
        "x.joystic.left" != name &&
        "y.joystic.left" != name
    {
        panic!("[manual_mode_common] [ERROR]: PRAMETER ERROR")
    }
}

