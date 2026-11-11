import unittest
from unittest.mock import patch
import sender

class HostButtonTests(unittest.TestCase):
    def test_remote_command_preserves_actual_A_index(self):
        command=sender.build_ssh_command('execute','a'*32,reset_button_index=3)
        self.assertIn('--gamepad-reset',command[-1])
        self.assertIn('--reset-button-index 3',command[-1])
        self.assertNotIn('reset.py --execute',command[-1])
    def test_bad_button_indices_cannot_enter_remote_shell(self):
        for value in [-1,13,True,'0; x']:
            with self.assertRaises(ValueError):
                sender.build_ssh_command('execute','a'*32,reset_button_index=value)
    def test_kernel_map_is_used_instead_of_a_fixed_index(self):
        def mapped(fd,request,codes):codes[3]=0x130
        with patch('fcntl.ioctl',side_effect=mapped):
            self.assertEqual(sender.a_button_index(100),3)
        with patch('fcntl.ioctl'):
            with self.assertRaises(ValueError):sender.a_button_index(100)

if __name__=='__main__':unittest.main()
