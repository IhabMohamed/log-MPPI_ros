
class MissionFileParser():

    """
    Define a parser for our custom mission syntax
    """

    def __init__(self, mission_file):
        # Parse the given file
        self.mission = self.__parse_file__(mission_file)

    def get_mission(self):
        """
        Return the parsed mission as python list
        """
        return self.mission

    def __parse_file__(self, mission_file):
        """
        Parse the given mission file
        """
        mission = list()

        # Open the file and read it line by line
        with open(mission_file, 'r') as file:

            for l in file:
                l = l.strip() # Remove whitespaces

                # First character is a #
                # Do not parse empty lines or comments
                if len(l) == 0 or l[0] == '#':
                    continue

                parsed = self.__parse_line__(l)
                if not parsed == None and not parsed[1] == None:
                    mission.append(parsed)
                else:
                    print('Error while parsing line: {}'.format(l))

        return mission

    def __parse_line__(self, line):
        """
        Parse a single line as one command
        """
        line_split = line.split(':')

        if len(line_split) == 2:
            # Check which command is defined in the line and call the matching 
            # command parser with the remainding part of the current line
            if line_split[0] == 'wp':
                return ('wp', self.__parse_waypoint__(line_split[1].strip()))
            elif line_split[0] == 'rd':
                return ('rd', self.__parse_random__(line_split[1].strip()))
            elif line_split[0] == 'cmd':
                return ('cmd', self.__parse_command__(line_split[1].strip()))
            else:
                print('Command not supported: {}'.format(line_split[0]))
                return None
        else:
            print('Line has no command')
            return None

    def __parse_waypoint__(self, input):
        """
        Parse the input string as single waypoint

        Returns:
            None if parsing fails and (x,y,yaw) otherwise
        """
        waypoint = [float(e) for e in input.split(' ')]
        if len(waypoint) == 3:
            return waypoint
        else:
            print('Waypoint has not 3 elements: {} given'.format(len(waypoint)))
            return None

    def __parse_random__(self, input):
        """
        Parse the input string as random waypoint command

        Returns:
            None if parsing fails and (N,min_x,max_x,min_y,max_y) otherwise
        """
        randomizer = [float(e) for e in input.split(' ')]
        if len(randomizer) == 5:
            return randomizer
        else:
            print('Random command has not 5 elements: {} given'.format(len(randomizer)))
            return None

    def __parse_command__(self, input):
        """
        Parse an arbitrary command

        Returns:
            The input string
        """
        return input.split('\n')[0]
